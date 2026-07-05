"""
Deterministic Mission Executor — the only layer that talks to the vehicle.
  • Receives only VALIDATED mission dicts.
  • The LLM is never imported here; this layer is completely isolated.
Uses pymavlink directly

Waypoints are flown one at a time in GUIDED mode via SET_POSITION_TARGET_GLOBAL_INT
setpoints, rather than uploaded as an AUTO mission list. This avoids the
MISSION_COUNT / MISSION_REQUEST / MISSION_ITEM upload handshake entirely, which
was consistently failing (MAV_MISSION_INVALID_SEQUENCE) against this SITL setup.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

import yaml
from pathlib import Path
from pymavlink import mavutil

from .mission_validator import haversine

_CONFIG_DIR = Path(__file__).parent.parent / "config"


def _load_safety() -> dict:
    with open(_CONFIG_DIR / "safety_config.yaml") as f:
        return yaml.safe_load(f)

MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178

MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

POSITION_TARGET_TYPEMASK_POSITION_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

MODE_GUIDED = "GUIDED"
MODE_LOITER = "LOITER"
MODE_RTL = "RTL"


@dataclass
class ExecutionStatus:
    mission_id: str
    state: str          
    current_wp: int = 0
    total_wp: int = 0
    loop: int = 0
    total_loops: int = 0
    message: str = ""


class MissionExecutor:
    def __init__(self, connection_string: Optional[str] = None):
        cfg = _load_safety()
        self._conn_str = connection_string or cfg["mavlink"]["connection_string"]
        self._timeout = cfg["mavlink"]["timeout_s"]
        self._mav: Optional[mavutil.mavfile] = None
        self._status = ExecutionStatus(mission_id="", state="idle")

    def connect(self):
        print(f"[Executor] Connecting to {self._conn_str} …")
        self._mav = mavutil.mavlink_connection(self._conn_str)
        hb = self._mav.wait_heartbeat(timeout=self._timeout)
        if hb is None or self._mav.target_system == 0:
            raise ConnectionError(
                f"No MAVLink heartbeat from {self._conn_str} within {self._timeout}s "
                f"(target_system={self._mav.target_system}).\n"
                f"  • Is the simulator running?\n"
                f"  • If using sim_vehicle.py/MAVProxy, connect to its UDP output "
                f"(udp:127.0.0.1:14550), NOT tcp:5760 — the autopilot's TCP port "
                f"serves only one client and MAVProxy already holds it."
            )
        print(f"[Executor] Connected — system {self._mav.target_system}, "
              f"component {self._mav.target_component}")
        self._request_data_streams(rate_hz=4)

    def _request_data_streams(self, rate_hz: int = 4):
        self._mav.mav.request_data_stream_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,          # start streaming
        )

    def execute(self, mission: dict, auto_arm: bool = False) -> ExecutionStatus:
        """
        Execute a validated mission dict.
        Steps: arm → takeoff → fly each waypoint in GUIDED → RTL/land.
        """
        if self._mav is None:
            self.connect()

        mid = mission["mission_id"]
        params = mission["parameters"]
        waypoints = mission["waypoints"]
        loops = params["loops"]
        alt = params["altitude_m"]
        speed = params["groundspeed_ms"]
        rtl = params["return_to_home"]

        self._status = ExecutionStatus(
            mission_id=mid,
            state="arming",
            total_wp=len(waypoints),
            total_loops=loops,
        )

        self._log(f"Starting mission {mid}")
        self._log(f"  {len(waypoints)} waypoints × {loops} loops @ {alt} m / {speed} m/s")
        self._set_speed(speed)

        # 2. Enter GUIDED first — ArduCopter arms cleanly here without RC,
        #    then wait until pre-arm (EKF/GPS) is satisfied.
        self._set_mode(MODE_GUIDED)
        self._wait_until_armable()

        # 3. Arm
        if auto_arm:
            self._arm()
        else:
            self._log("Waiting for manual arm … (set auto_arm=True to skip)")
            self._wait_armed()

        # 4. Takeoff in GUIDED mode
        self._status.state = "takeoff"
        self._guided_takeoff(alt)

        # 4. Fly each waypoint in GUIDED mode, one at a time
        self._status.state = "executing"
        self._fly_waypoints_guided(waypoints, alt, loops)

        # 5. Switch to RTL and wait for landing
        if rtl:
            self._status.state = "rtl"
            self._log("Mission complete — switching to RTL")
            self._set_mode(MODE_RTL)
            self._wait_landed()

        self._status.state = "done"
        self._log("Mission done.")
        return self._status

    def _wait_until_armable(self, timeout: float = 60):
        """Wait for a GPS 3D fix so arming passes ArduCopter's pre-arm checks."""
        self._log("Waiting for GPS/EKF to be ready …")
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
            if msg and msg.fix_type >= 3:
                self._log(f"GPS 3D fix acquired ({msg.satellites_visible} sats).")
                return
        self._log("WARNING: proceeding without a confirmed 3D fix (timed out).")

    def _arm(self, timeout: float = 30):
        """
        Arm the vehicle with a bounded wait. Re-sends the arm command periodically
        and surfaces any PreArm STATUSTEXT so failures are visible, never silent.
        """
        self._log("Arming …")
        self._mav.arducopter_arm()
        deadline = time.time() + timeout
        last_retry = time.time()
        while time.time() < deadline:
            msg = self._mav.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                if "Arm" in msg.text or "EKF" in msg.text or "GPS" in msg.text:
                    self._log(f"  [autopilot] {msg.text}")
                continue
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                self._log("Armed.")
                return
            # Retry the arm command every few seconds while pre-arm clears
            if time.time() - last_retry > 3:
                self._mav.arducopter_arm()
                last_retry = time.time()
        raise TimeoutError("Failed to arm within timeout — see PreArm messages above")

    def _wait_armed(self, timeout: float = 120):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self._log("Detected armed state.")
                return
        raise TimeoutError("Timed out waiting for arm")

    def _send_takeoff(self, alt_m: float):
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m,
        )

    def _guided_takeoff(self, alt_m: float):
        self._set_mode(MODE_GUIDED)
        self._log(f"Taking off to {alt_m} m …")
        self._send_takeoff(alt_m)
        # Poll altitude; if the copter hasn't begun climbing, re-issue the
        # takeoff command. This is robust against a command being dropped or
        # arriving in the brief window before the autopilot is takeoff-ready.
        deadline = time.time() + 60
        last_resend = time.time()
        while time.time() < deadline:
            msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            if msg:
                rel_alt = msg.relative_alt / 1000.0
                if abs(rel_alt - alt_m) <= 2.0:
                    self._log(f"Reached {alt_m} m.")
                    return
                if rel_alt < 1.0 and time.time() - last_resend > 4:
                    self._log("  (re-issuing takeoff — no climb detected)")
                    self._send_takeoff(alt_m)
                    last_resend = time.time()
        raise TimeoutError(f"Timed out reaching {alt_m} m altitude")

    def _set_speed(self, speed_ms: float):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            MAV_CMD_DO_CHANGE_SPEED,
            0, 1, speed_ms, -1, 0, 0, 0, 0,
        )

    def _set_mode(self, mode_name: str, timeout: float = 15):
        mode_id = self._mav.mode_mapping()[mode_name]
        deadline = time.time() + timeout
        last_send = 0.0
        while time.time() < deadline:
            # Re-issue the command periodically — a single set_mode can be
            # dropped if it lands amid other traffic (e.g. just after arming).
            if time.time() - last_send > 2:
                self._mav.set_mode(mode_id)
                last_send = time.time()
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and mode_name in mavutil.mode_string_v10(msg):
                return
        raise TimeoutError(f"Mode change to {mode_name} timed out")

    def _fly_waypoints_guided(
        self,
        waypoints: list,
        cruise_alt: float,
        loops: int,
        wp_radius_m: float = 2.0,
    ):
        """
        Fly each waypoint sequentially in GUIDED mode using position setpoints.
        No mission upload — the vehicle only ever knows about the ONE waypoint
        it's currently flying to.
        """
        total = len(waypoints) * loops
        count = 0
        for loop in range(loops):
            self._status.loop = loop + 1
            for wp in waypoints:
                count += 1
                alt = wp.get("alt_m", cruise_alt)
                self._status.current_wp = count
                self._log(
                    f"  → WP {count}/{total}: lat={wp['lat']:.6f} "
                    f"lon={wp['lon']:.6f} alt={alt} m"
                )
                self._goto_position(wp["lat"], wp["lon"], alt)
                self._wait_reached(wp["lat"], wp["lon"], alt, radius_m=wp_radius_m)
                hold = wp.get("hold_s", 0)
                if hold:
                    self._log(f"    Holding {hold}s …")
                    time.sleep(hold)
        self._log("All waypoints reached.")

    def _goto_position(self, lat: float, lon: float, alt_m: float):
        self._mav.mav.set_position_target_global_int_send(
            0,  # time_boot_ms — unused
            self._mav.target_system, self._mav.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_TARGET_TYPEMASK_POSITION_ONLY,
            int(lat * 1e7), int(lon * 1e7), alt_m,
            0, 0, 0,     # vx, vy, vz — ignored
            0, 0, 0,     # afx, afy, afz — ignored
            0, 0,        # yaw, yaw_rate — ignored
        )

    def _wait_reached(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        radius_m: float = 2.0,
        alt_tolerance_m: float = 2.0,
        timeout: float = 120,
        resend_period_s: float = 0.5,
    ):
        """
        Wait until the vehicle is within radius_m/alt_tolerance_m of the target,
        re-sending the setpoint at resend_period_s — ArduPilot reverts to LOITER
        if GUIDED setpoints stop arriving for too long (external-control timeout).
        """
        deadline = time.time() + timeout
        last_send = 0.0
        while time.time() < deadline:
            if time.time() - last_send > resend_period_s:
                self._goto_position(lat, lon, alt_m)
                last_send = time.time()
            msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if msg:
                cur_lat = msg.lat / 1e7
                cur_lon = msg.lon / 1e7
                cur_alt = msg.relative_alt / 1000.0
                dist = haversine(cur_lat, cur_lon, lat, lon)
                if dist <= radius_m and abs(cur_alt - alt_m) <= alt_tolerance_m:
                    return
        raise TimeoutError(
            f"Timed out flying to waypoint (lat={lat}, lon={lon}, alt={alt_m} m)"
        )

    def _wait_altitude(self, target_m: float, tolerance: float = 2.0, timeout: float = 60):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            if msg:
                rel_alt = msg.relative_alt / 1000.0
                if abs(rel_alt - target_m) <= tolerance:
                    return
        raise TimeoutError(f"Timed out reaching {target_m} m altitude")

    def _wait_landed(self, timeout: float = 120):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg:
                mode = mavutil.mode_string_v10(msg)
                if "LAND" in mode:
                    # Wait for disarm
                    if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        self._log("Landed and disarmed.")
                        return

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] [Executor] {msg}")
        self._status.message = msg
