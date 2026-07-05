#!/usr/bin/env bash
set -e

# Start Gazebo in background
gz sim -r /opt/ardupilot_gazebo/worlds/iris_runway.sdf &
GZ_PID=$!
sleep 5

# Start ArduPilot SITL in background
cd /opt/ardupilot
Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --no-rebuild \
  --out=udp:127.0.0.1:14550 &
SITL_PID=$!
sleep 8

# Run the demo
cd /app
exec "$@"

# Cleanup
kill $GZ_PID $SITL_PID 2>/dev/null || true
