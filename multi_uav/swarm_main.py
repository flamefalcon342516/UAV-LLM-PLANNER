#!/usr/bin/env python3
"""
Omokai UAV — Swarm Formation Demo
===================================
  Prompt → LLM → Swarm JSON → Validator → Formation Planner → Swarm Executor → N SITL vehicles

Usage:
  python3 swarm_main.py
  python3 swarm_main.py --prompt "Fly 3 drones in wedge formation at 20 metres"
  python3 swarm_main.py --prompt "Two drones side by side patrol at 15m" --auto-arm
  python3 swarm_main.py --load missions/examples/wedge_3drones.json --auto-arm
  python3 swarm_main.py --prompt "..." --dry-run
  python3 swarm_main.py --load missions/examples/line_2drones.json --dry-run
"""

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.swarm_llm_planner  import plan_swarm_mission, DEFAULT_MODEL
from src.swarm_validator    import validate_swarm_mission, SwarmValidationError
from src.formation_planner  import plan_formation
from src.swarm_executor     import SwarmExecutor, SwarmExecutionError

_CONNECTIONS = {
    "copter_1": "udp:127.0.0.1:14550",
    "copter_2": "udp:127.0.0.1:14560",
    "copter_3": "udp:127.0.0.1:14570",
}

def banner() -> None:
    print("""
          PIPELINE STARTS
""")


def print_swarm_mission(mission: dict) -> None:
    f = mission["formation"]
    p = mission["parameters"]
    print("\n── Swarm Mission ──────────────────────────────────────────────")
    print(f"  ID         : {mission['mission_id']}")
    print(f"  Input      : {mission.get('natural_language_input', '')}")
    print(f"  Formation  : {f['type']}  (spacing {f['spacing_m']} m)")
    print(f"  Leader     : {f['leader_id']}")
    print(f"  Vehicles   : {mission['vehicles']}")
    print(f"  Altitude   : {p['altitude_m']} m")
    print(f"  Speed      : {p['groundspeed_ms']} m/s")
    print(f"  Loops      : {p['loops']}")
    print(f"  RTH        : {p['return_to_home']}")
    print(f"  Waypoints  : {len(mission['waypoints'])} (leader route)")
    for wp in mission["waypoints"]:
        print(
            f"    WP{wp['id']:02d}  "
            f"lat={wp['lat']:.6f}  lon={wp['lon']:.6f}  "
            f"alt={wp['alt_m']} m  action={wp.get('action', 'none')}"
        )
    print("───────────────────────────────────────────────────────────────\n")

def print_per_drone_missions(missions: list) -> None:
    print("── Per-Drone Missions (Formation Offsets Applied) ─────────────")
    for m in missions:
        vid = m["vehicle_id"]
        print(f"\n  [{vid}]   {len(m['waypoints'])} waypoints")
        for wp in m["waypoints"]:
            print(
                f"    WP{wp['id']:02d}  "
                f"lat={wp['lat']:.6f}  lon={wp['lon']:.6f}  "
                f"alt={wp['alt_m']} m"
            )
    print("\n───────────────────────────────────────────────────────────────\n")

def confirm(prompt: str = "Execute swarm mission? [y/N] ") -> bool:
    try:
        return input(prompt).strip().lower() == "y"
    except (EOFError, KeyboardInterrupt):
        return False

def main() -> None:
    banner()

    parser = argparse.ArgumentParser(description="Omokai UAV Swarm Demo")
    parser.add_argument(
        "--prompt", type=str,
        help="Natural-language swarm mission instruction",
    )
    parser.add_argument(
        "--auto-arm", action="store_true",
        help="Arm all vehicles automatically (no manual confirmation per vehicle)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Plan, validate, and print the formation — do not connect or fly",
    )
    parser.add_argument(
        "--model", type=str, default=DEFAULT_MODEL,
        help=f"LLM model ID (default: {DEFAULT_MODEL})",
    )
    parser.add_argument(
        "--load", type=str, metavar="FILE",
        help="Load a pre-built swarm mission JSON instead of calling the LLM",
    )
    parser.add_argument(
        "--save", type=str, metavar="FILE",
        help="Write the validated swarm mission JSON to FILE",
    )
    args = parser.parse_args()

    if not args.load and not os.environ.get("LLM_API_KEY"):
        print("ERROR: LLM_API_KEY environment variable is not set.")
        print("  export LLM_API_KEY=sk-live-...")
        sys.exit(1)

    if args.load:
        print(f"Loading swarm mission from {args.load} …")
        with open(args.load) as f:
            swarm_mission = json.load(f)
        print("Skipping LLM (pre-built mission loaded).\n")
    else:
        prompt = args.prompt
        if not prompt:
            print("Enter a swarm mission instruction.  Examples:")
            print('  "Fly 3 drones in wedge formation at 20 metres"')
            print('  "Two drones patrol side by side at 15 metres"')
            print('  "Three drones in column formation — patrol the perimeter"')
            print('  "Drone 1 inspect waypoint A, drone 2 patrol waypoint B"')
            prompt = input("\n>>> ").strip()
            if not prompt:
                print("No input provided. Exiting.")
                sys.exit(0)

        print(f"\n[1/4] Sending prompt to LLM ({args.model}) …")
        try:
            swarm_mission = plan_swarm_mission(prompt, model=args.model)
            print("\n── Raw LLM Output ─────────────────────────────────────────────")
            print(json.dumps(swarm_mission, indent=2))
            print("───────────────────────────────────────────────────────────────\n")
        except Exception as exc:
            print(f"LLM error: {exc}")
            sys.exit(1)
            
    print("[2/4] Validating swarm mission …")
    try:
        swarm_mission = validate_swarm_mission(swarm_mission)
        print("      ✓ All safety checks passed.\n")
    except SwarmValidationError as exc:
        print(f"\n      ✗ Mission REJECTED:\n{exc}")
        sys.exit(1)

    print_swarm_mission(swarm_mission)

    if args.save:
        with open(args.save, "w") as f:
            json.dump(swarm_mission, f, indent=2)
        print(f"Mission saved to {args.save}\n")

    print("[3/4] Running formation planner …")
    try:
        per_drone_missions = plan_formation(swarm_mission)
        n = len(per_drone_missions)
        print(f"      ✓ Generated {n} individual mission(s).\n")
    except Exception as exc:
        print(f"      ✗ Formation planner error: {exc}")
        sys.exit(1)

    print_per_drone_missions(per_drone_missions)
    if args.dry_run:
        print("[Dry run] Planning complete — not connecting or executing.")
        sys.exit(0)

    vehicles    = swarm_mission["vehicles"]
    connections = [_CONNECTIONS[vid] for vid in vehicles]

    print(f"[4/4] Preparing to connect to {len(vehicles)} SITL instance(s):")
    for vid, conn in zip(vehicles, connections):
        print(f"        {vid}  →  {conn}")
    print()

    if not args.auto_arm and not confirm():
        print("Mission cancelled.")
        sys.exit(0)

    executor = SwarmExecutor(connections)
    try:
        results = executor.execute_swarm(per_drone_missions, auto_arm=args.auto_arm)
    except SwarmExecutionError as exc:
        print(f"\nSwarm execution failed:\n{exc}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nMission aborted by operator.")
        sys.exit(1)

    print("\n── Swarm Results ──────────────────────────────────────────────")
    all_ok = True
    for r in results:
        icon = "✓" if r.status.state == "done" else "✗"
        err  = f"  ({r.error})" if r.error else ""
        print(f"  {icon} {r.vehicle_id}  state={r.status.state}{err}")
        if r.status.state != "done":
            all_ok = False
    print("───────────────────────────────────────────────────────────────")
    print("Formation mission complete.\n" if all_ok else "Mission completed with errors.\n")


if __name__ == "__main__":
    main()
