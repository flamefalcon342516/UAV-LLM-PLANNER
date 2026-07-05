"""
Target Follower — switches drone to GUIDED mode and issues velocity
setpoints to keep the detected target centred in the camera frame.

Control law:
  - Pixel error (target_cx - frame_cx) drives yaw rate (proportional).
  - Target always present → hold position; target lost → loiter.
  - Altitude held constant (no vertical tracking in this demo).

This is a simple proportional controller; a production system would use
a Kalman filter on the target position and a full PID on yaw + forward velocity.
"""

import time
import math
import threading
from typing import Optional, TYPE_CHECKING

from pymavlink import mavutil

if TYPE_CHECKING:
    from .detector import TargetDetector


class TargetFollower:
    """
    Attaches to a live MAVLink connection and a TargetDetector.
    When a detection event arrives it:
      1. Switches to GUIDED mode.
      2. Issues NED velocity setpoints to move toward the target.
      3. Returns to LOITER if the target is lost for > lost_timeout seconds.
    """

    # Proportional gain: pixels → m/s lateral correction
    KP_YAW = 0.005       # deg/s per pixel error
    KP_FWD = 0.3         # m/s forward when target detected
    MAX_YAW_RATE = 30    # deg/s
    MAX_SPEED = 3        # m/s

    def __init__(self, mav: mavutil.mavfile, lost_timeout: float = 3.0):
        self._mav = mav
        self._lost_timeout = lost_timeout
        self._running = False
        self._last_detection: Optional[dict] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._following = False

    def on_detection(self, event: dict):
        """Callback registered with TargetDetector."""
        with self._lock:
            self._last_detection = event
            if not self._following:
                self._start_follow()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)

    # ------------------------------------------------------------------

    def _start_follow(self):
        if self._following:
            return
        self._following = True
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        print("[Follower] Target acquired — switching to GUIDED follow mode")

    def _control_loop(self):
        self._set_mode("GUIDED")
        last_seen = time.time()

        while self._running:
            with self._lock:
                det = self._last_detection
                self._last_detection = None

            if det is not None:
                last_seen = time.time()
                self._send_velocity(det)
            elif time.time() - last_seen > self._lost_timeout:
                print("[Follower] Target lost — switching to LOITER")
                self._set_mode("LOITER")
                self._following = False
                self._running = False
                break

            time.sleep(0.1)

    def _send_velocity(self, det: dict):
        h, w = det["frame_shape"][:2]
        cx, cy = det["center"]
        frame_cx = w / 2

        # Pixel error → yaw rate
        px_err = cx - frame_cx
        yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, self.KP_YAW * px_err))

        # Always move forward toward target
        vx = self.KP_FWD  # forward (body frame → NED transform would be needed for full 3D)
        vy = 0.0
        vz = 0.0

        # SET_POSITION_TARGET_LOCAL_NED with velocity mask
        type_mask = (
            0b0000_1111_1100_0111   # ignore pos, accel, yaw; use velocity + yaw_rate
        )

        self._mav.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b110111000111,         # use vx, vy, vz, yaw_rate
            0, 0, 0,                # position (ignored)
            vx, vy, vz,             # velocity m/s
            0, 0, 0,                # acceleration (ignored)
            0,                      # yaw (ignored)
            math.radians(yaw_rate), # yaw_rate rad/s
        )

    def _set_mode(self, mode_name: str):
        try:
            mode_id = self._mav.mode_mapping()[mode_name]
            self._mav.set_mode(mode_id)
        except Exception as e:
            print(f"[Follower] Mode set failed: {e}")
