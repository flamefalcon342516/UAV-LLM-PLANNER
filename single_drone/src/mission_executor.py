"""
Deterministic Mission Executor — the only layer that talks to the vehicle.

Contract:
  • Receives only VALIDATED mission dicts.
  • The same JSON always produces the same MAVLink command sequence.
  • The LLM is never imported here; this layer is completely isolated.
  • All decisions are rule-based: read JSON → issue MAVLink → wait → repeat.

Uses pymavlink directly for maximum portability (no ROS dependency).
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

import yaml
from pathlib import Path
from pymavlink import mavutil

_CONFIG_DIR = Path(__file__).parent.parent / "config"


def _load_safety() -> dict:
    with open(_CONFIG_DIR / "safety_config.yaml") as f:
        return yaml.safe_load(f)

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178

MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

MODE_GUIDED = "GUIDED"
MODE_AUTO = "AUTO"
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
            1,  # start streaming
        )

    def execute(self, mission: dict, auto_arm: bool = False) -> ExecutionStatus:
        """
        Execute a validated mission dict.
        Steps: arm → takeoff → upload waypoints → AUTO → monitor → RTL/land.
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

        # 1. Set ground speed
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

        # 4. Build and upload mission for AUTO mode
        self._status.state = "executing"
        self._upload_mission(waypoints, alt, speed, loops, rtl)

        # 5. Switch to AUTO
        self._set_mode(MODE_AUTO)
        self._log("AUTO mode set — mission executing")

        # 6. Monitor progress
        self._monitor_mission(len(waypoints) * loops)

        # 7. Wait for RTL / landing
        if rtl:
            self._status.state = "rtl"
            self._log("Mission complete — RTL")
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

    def _upload_mission(
        self,
        waypoints: list,
        cruise_alt: float,
        speed_ms: float,
        loops: int,
        rtl: bool,
    ):
        """Build and upload MAVLink mission items from the JSON waypoints."""
        items = []
        wp_idx = 0

        # Item 0: dummy home (required by ArduPilot as item 0)
        items.append(
            self._mav.mav.mission_item_int_encode(
                self._mav.target_system,
                self._mav.target_component,
                wp_idx, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                0, 0, 0,  # lat/lon/alt as 0 = use current
            )
        )
        wp_idx += 1

        # Repeat waypoints for the requested number of loops
        for loop in range(loops):
            for wp in waypoints:
                hold = wp.get("hold_s", 0)
                action = wp.get("action", "none")
                cmd = (
                    MAV_CMD_NAV_LOITER_TIME if action == "loiter"
                    else MAV_CMD_NAV_WAYPOINT
                )
                items.append(
                    self._mav.mav.mission_item_int_encode(
                        self._mav.target_system,
                        self._mav.target_component,
                        wp_idx,
                        MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        cmd,
                        0, 1,
                        hold, 0, 0, 0,
                        int(wp["lat"] * 1e7),
                        int(wp["lon"] * 1e7),
                        wp.get("alt_m", cruise_alt),
                    )
                )
                wp_idx += 1

        if rtl:
            items.append(
                self._mav.mav.mission_item_int_encode(
                    self._mav.target_system,
                    self._mav.target_component,
                    wp_idx, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                )
            )

        self._log(f"Uploading {len(items)} mission items …")

        self._mav.mav.mission_count_send(
            self._mav.target_system,
            self._mav.target_component,
            len(items),
        )

        # Handshake: GCS sends items in response to MISSION_REQUEST
        ack_type = None
        expected_idx = 0
        for _ in range(len(items) * 3):
            msg = self._mav.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                blocking=True,
                timeout=5,
            )
            if msg is None:
                break
            if msg.get_type() == "MISSION_ACK":
                ack_type = msg.type
                break
            idx = msg.seq
            self._log(
                f"  [debug] got {msg.get_type()} seq={idx} "
                f"(expected {expected_idx}); sending item seq={items[idx].seq if 0 <= idx < len(items) else 'N/A'}"
            )
            if idx < len(items):
                self._mav.mav.send(items[idx])
                expected_idx = idx + 1

        if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            got = "no ACK (timed out)" if ack_type is None else f"ack type {ack_type}"
            raise RuntimeError(
                f"Mission upload failed — {got}. The autopilot likely reported "
                f"'mission upload timeout' on its console (a dropped MAVLink packet "
                f"over the UDP relay is the usual cause). Retry the mission."
            )

        self._log("Mission upload complete.")

    def _monitor_mission(self, total_wp: int, timeout: float = 600):
        deadline = time.time() + timeout
        last_wp = -1
        while time.time() < deadline:
            msg = self._mav.recv_match(
                type=["MISSION_ITEM_REACHED", "MISSION_CURRENT", "HEARTBEAT"],
                blocking=True,
                timeout=2,
            )
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "MISSION_ITEM_REACHED":
                seq = msg.seq
                if seq != last_wp:
                    last_wp = seq
                    self._status.current_wp = seq
                    self._log(f"  → Reached waypoint {seq}/{total_wp}")
                    if seq >= total_wp:
                        return
            elif mtype == "HEARTBEAT":
                mode = mavutil.mode_string_v10(msg)
                if "RTL" in mode or "LAND" in mode:
                    self._log(f"  Mode switched to {mode} — mission complete")
                    return
        raise TimeoutError(
            f"Mission stalled — last progress was waypoint {last_wp}/{total_wp} "
            f"({timeout:.0f}s elapsed with no further progress)."
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
