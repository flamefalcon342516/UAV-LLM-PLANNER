#!/usr/bin/env python3
"""
Omokai UAV Demo — end-to-end pipeline
======================================
  Prompt → LLM → Validated Mission JSON → Deterministic Executor → ArduPilot SITL

Usage:
  python3 main.py                         # Interactive prompt
  python3 main.py --prompt "Patrol the perimeter twice at 15 metres"
  python3 main.py --prompt "..." --auto-arm      # Skip manual arm
  python3 main.py --vision person                # Enable vision + follow
  python3 main.py --dry-run                      # Plan + validate only, no fly
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from src.llm_planner import plan_mission, DEFAULT_MODEL
from src.mission_validator import validate_mission, ValidationError
from src.mission_executor import MissionExecutor


def banner():
    print(""" PIPELINE DEMO
""")


def print_mission(mission: dict):
    print("\n── Proposed Mission ─────────────────────────────────")
    print(f"  ID        : {mission['mission_id']}")
    print(f"  Input     : {mission.get('natural_language_input', '')}")
    print(f"  Altitude  : {mission['parameters']['altitude_m']} m")
    print(f"  Speed     : {mission['parameters']['groundspeed_ms']} m/s")
    print(f"  Loops     : {mission['parameters']['loops']}")
    print(f"  RTH       : {mission['parameters']['return_to_home']}")
    print(f"  Waypoints : {len(mission['waypoints'])}")
    for wp in mission["waypoints"]:
        print(f"    WP{wp['id']:02d}  lat={wp['lat']:.6f}  lon={wp['lon']:.6f}  alt={wp['alt_m']} m  action={wp.get('action','none')}")
    print("─────────────────────────────────────────────────────\n")


def confirm(prompt: str = "Execute this mission? [y/N] ") -> bool:
    try:
        return input(prompt).strip().lower() == "y"
    except (EOFError, KeyboardInterrupt):
        return False


def main():
    banner()

    parser = argparse.ArgumentParser(description="Omokai UAV Demo")
    parser.add_argument("--prompt", type=str, help="Natural-language mission instruction")
    parser.add_argument("--auto-arm", action="store_true", help="Arm automatically (no manual confirmation)")
    parser.add_argument("--dry-run", action="store_true", help="Plan and validate only — do not execute")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL, help="Claude model ID (e.g. claude-sonnet-4-6)")
    parser.add_argument("--save", type=str, metavar="FILE", help="Save validated mission JSON to FILE")
    parser.add_argument("--load", type=str, metavar="FILE", help="Load mission from FILE instead of LLM")
    parser.add_argument("--connect", type=str, metavar="CONN", default=None,
                        help="MAVLink connection string override (e.g. tcp:127.0.0.1:5760). "
                             "Defaults to the value in config/safety_config.yaml.")
    args = parser.parse_args()

    if not args.load and not os.environ.get("LLM_API_KEY"):
        print("ERROR: LLM_API_KEY environment variable is not set.")
        print("export LLM_API_KEY=...")
        sys.exit(1)

    if args.load:
        print(f"Loading mission from {args.load} …")
        with open(args.load) as f:
            mission = json.load(f)
        print("Skipping LLM (using saved mission).")
    else:
        prompt = args.prompt
        if not prompt:
            print("Enter your mission instruction (e.g. 'Patrol the perimeter loop twice at 15 metres'):")
            prompt = input(">>> ").strip()
            if not prompt:
                print("No input provided. Exiting.")
                sys.exit(0)

        print(f"\n[1/3] Sending prompt to LLM ({args.model}) …")
        max_attempts = 3
        retry_delay_s = 2
        mission = None
        last_error = None
        for attempt in range(1, max_attempts + 1):
            try:
                mission = plan_mission(prompt, model=args.model)
                last_error = None
                break
            except Exception as e:
                last_error = e
                if attempt < max_attempts:
                    print(f"      LLM call failed (attempt {attempt}/{max_attempts}): {e}")
                    print(f"      Retrying in {retry_delay_s}s …")
                    time.sleep(retry_delay_s)

        if last_error is not None:
            print(f"LLM error: {last_error}")
            sys.exit(1)

        print("\n========== RAW LLM OUTPUT ==========\n")
        print(json.dumps(mission, indent=2))
        print("\n====================================\n")
            
    with open("llm_raw_output.json", "w") as f:
        json.dump(mission, f, indent=2)         

    print("[2/3] Validating mission …")
    try:
        mission = validate_mission(mission)
        print("      ✓ Mission passed all safety checks.")
    except ValidationError as e:
        print(f"\n      ✗ Mission REJECTED:\n{e}")
        sys.exit(1)

    print_mission(mission)

    if args.save:
        with open(args.save, "w") as f:
            json.dump(mission, f, indent=2)
        print(f"Mission saved to {args.save}")

    if args.dry_run:
        print("[Dry run] Validation complete — not executing.")
        sys.exit(0)

    if not args.auto_arm:
        if not confirm():
            print("Mission cancelled.")
            sys.exit(0)

    executor = MissionExecutor(connection_string=args.connect)
    executor.connect()

    print("[3/3] Executing mission …\n")
    try:
        status = executor.execute(mission, auto_arm=args.auto_arm)
        print(f"\nFinal status: {status.state}")
    except KeyboardInterrupt:
        print("\nMission aborted by operator.")
    except Exception as e:
        print(f"Execution error: {e}")
        raise


if __name__ == "__main__":
    main()
