"""
Swarm Executor — connects to N ArduPilot SITL instances and flies them in formation.
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

                # ── Phase 5: Fly each waypoint in GUIDED mode ────────────────
                ex._status.state = "executing"
                self._log(idx, vid, "Flying waypoints (GUIDED) …")
                ex._fly_waypoints_guided(
                    mission["waypoints"],
                    alt,
                    params["loops"],
                )
                self._log(idx, vid, "Waypoints complete.")
                sync("complete")

                # ── Phase 6: RTL + wait landed ───────────────────────────────
                if params.get("return_to_home", True):
                    ex._status.state = "rtl"
                    self._log(idx, vid, "RTL — waiting for landing …")
                    ex._set_mode("RTL")
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
