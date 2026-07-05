#!/usr/bin/env bash
# Launch a plain ArduPilot SITL instance (no Gazebo).
# Run this BEFORE main.py
# Usage: ./sim/launch_sitl.sh

set -e
ARDUPILOT_DIR="${HOME}/ardupilot"

echo "[SITL] Starting ArduPilot SITL …"
echo "[SITL] MAVLink will be available on udp:127.0.0.1:14550"
echo ""

cd "${ARDUPILOT_DIR}"
Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  --console \
  --map \
  --no-rebuild \
  --out=udp:127.0.0.1:14550
