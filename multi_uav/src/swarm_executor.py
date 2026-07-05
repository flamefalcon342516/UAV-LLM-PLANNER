"""
Swarm Executor — connects to N ArduPilot SITL instances and flies them in formation.

Design:
  • Pure pymavlink — no dronekit, no ROS dependency.
  • Each vehicle is driven by its own MissionExecutor instance.
  • N threads run in lockstep, synchronized by threading.Barrier at every phase.
  • If any vehicle fails at any phase, barrier.abort() unblocks all other threads
    cleanly via BrokenBarrierError — no hung threads.
  • The same per-drone missions always produce the same MAVLink sequence.
  • The LLM is never imported here.

Phases (all synchronized):
    1. connect
    2. wait GPS/EKF (armable)
    3. arm
    4. guided takeoff
    5. upload mission + switch AUTO
    6. monitor mission completion
    7. RTL + wait landed
"""

import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from single_drone.src.mission_executor import MissionExecutor, ExecutionStatus


@dataclass
class SwarmStatus:
    vehicle_id:  str
    connection:  str
    status:      ExecutionStatus
    error:       Optional[str] = None


class SwarmExecutionError(Exception):
    pass


class SwarmExecutor:
    """
    Flies N drones through their individually-computed missions, keeping all
    vehicles phase-synchronized via threading.Barrier checkpoints.
    """

    # Maximum seconds any vehicle may spend in a single phase before the
    # barrier raises BrokenBarrierError and aborts the entire swarm.
    _PHASE_TIMEOUT = 240.0

    def __init__(self, connection_strings: List[str]):
        if not connection_strings:
            raise ValueError("At least one connection string is required.")
        self._connections = connection_strings
        self._n = len(connection_strings)

    def execute_swarm(
        self,
        per_drone_missions: List[dict],
        auto_arm: bool = False,
    ) -> List[SwarmStatus]:
        """
        Execute one validated mission per drone, synchronized at each flight phase.

        Args:
            per_drone_missions: Per-drone mission dicts from FormationPlanner.plan().
                                Must be in the same order as connection_strings.
            auto_arm:           True = arm automatically; False = wait for manual arm.

        Returns:
            List[SwarmStatus], one per vehicle, in input order.

        Raises:
            SwarmExecutionError if any vehicle fails.
        """
        if len(per_drone_missions) != self._n:
            raise ValueError(
                f"Received {len(per_drone_missions)} missions "
                f"but have {self._n} connection strings."
            )

        executors = [MissionExecutor(conn) for conn in self._connections]
        barrier   = threading.Barrier(self._n)
        errors: dict  = {}
        lock          = threading.Lock()
        results: List[Optional[SwarmStatus]] = [None] * self._n

        def fly(idx: int) -> None:
            ex      = executors[idx]
            mission = per_drone_missions[idx]
            vid     = mission.get("vehicle_id", f"drone_{idx + 1}")
            conn    = self._connections[idx]
            params  = mission["parameters"]
            alt     = float(params["altitude_m"])

            def sync(label: str) -> None:
                """Wait at the phase barrier; propagate abort cleanly."""
                try:
                    barrier.wait(timeout=self._PHASE_TIMEOUT)
                except threading.BrokenBarrierError:
                    raise  # caught by the outer except

            try:
                # ── Phase 1: Connect ────────────────────────────────────────
                self._log(idx, vid, "Connecting …")
                ex.connect()
                sync("connect")

                # ── Phase 2: Set speed + wait GPS/EKF ──────────────────────
                ex._set_speed(params["groundspeed_ms"])
                ex._set_mode("GUIDED")
                self._log(idx, vid, "Waiting for GPS fix …")
                ex._wait_until_armable()
                sync("armable")

                # ── Phase 3: Arm ────────────────────────────────────────────
                if auto_arm:
                    self._log(idx, vid, "Arming …")
                    ex._arm()
                else:
                    self._log(idx, vid, "Waiting for manual arm …")
                    ex._wait_armed()
                sync("armed")

                # ── Phase 4: Guided takeoff ─────────────────────────────────
                ex._status.state = "takeoff"
                self._log(idx, vid, f"Taking off to {alt} m …")
                ex._guided_takeoff(alt)
                sync("altitude")

                # ── Phase 5: Upload mission + switch to AUTO ────────────────
                ex._status.state = "executing"
                self._log(idx, vid, "Uploading mission …")
                ex._upload_mission(
                    mission["waypoints"],
                    alt,
                    params["groundspeed_ms"],
                    params["loops"],
                    params["return_to_home"],
                )
                ex._set_mode("AUTO")
                self._log(idx, vid, "AUTO — formation flying …")
                sync("auto")

                # ── Phase 6: Monitor mission completion ─────────────────────
                total_wp = len(mission["waypoints"]) * params["loops"]
                ex._monitor_mission(total_wp)
                sync("complete")

                # ── Phase 7: RTL + wait landed ──────────────────────────────
                if params.get("return_to_home", True):
                    ex._status.state = "rtl"
                    self._log(idx, vid, "RTL — waiting for landing …")
                    ex._wait_landed()

                ex._status.state = "done"
                self._log(idx, vid, "Done.")
                results[idx] = SwarmStatus(
                    vehicle_id=vid, connection=conn, status=ex._status
                )

            except threading.BrokenBarrierError:
                ex._status.state = "aborted"
                ex._status.message = "Aborted — another vehicle failed"
                results[idx] = SwarmStatus(
                    vehicle_id=vid,
                    connection=conn,
                    status=ex._status,
                    error="BrokenBarrier",
                )

            except Exception as exc:
                with lock:
                    errors[idx] = exc
                ex._status.state = "aborted"
                ex._status.message = str(exc)
                results[idx] = SwarmStatus(
                    vehicle_id=vid,
                    connection=conn,
                    status=ex._status,
                    error=str(exc),
                )
                try:
                    barrier.abort()
                except Exception:
                    pass

        threads = [
            threading.Thread(
                target=fly,
                args=(i,),
                name=f"swarm-drone-{i}",
                daemon=True,
            )
            for i in range(self._n)
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        if errors:
            lines = [f"  Drone {i + 1} ({self._connections[i]}): {e}"
                     for i, e in sorted(errors.items())]
            raise SwarmExecutionError(
                "Swarm execution failed:\n" + "\n".join(lines)
            )

        return [r for r in results if r is not None]

    @staticmethod
    def _log(idx: int, vid: str, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] [D{idx + 1}/{vid}] {msg}")
