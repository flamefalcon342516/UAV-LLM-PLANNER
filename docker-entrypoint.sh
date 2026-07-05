#!/usr/bin/env bash
set -e

# Launch ArduCopter SITL and MAVProxy directly, bypassing sim_vehicle.py's own
# wrapper. Verified: sim_vehicle.py's process supervision kills MAVProxy
# within seconds inside this container every time ("MAVProxy exited"), while
# launching the same two processes directly runs indefinitely without issue.

# MAVProxy's --out links always connect in send/client mode, so a viewer link
# needs a real destination address, not 0.0.0.0. To reach the host, send to
# the container's default gateway (the docker0 bridge IP) — no -p publish
# needed, since this is the container proactively reaching the host, not the
# other way round.
GATEWAY_IP=$(python3 -c "
import socket, struct
with open('/proc/net/route') as f:
    for line in f.readlines()[1:]:
        fields = line.strip().split()
        if fields[1] == '00000000':
            print(socket.inet_ntoa(struct.pack('<L', int(fields[2], 16))))
            break
")

launch_drone() {
  local instance=$1 sysid=$2 out_port=$3
  local tcp_port=$((5760 + instance * 10))

  /opt/ardupilot/build/sitl/bin/arducopter \
    --model + --speedup 1 --slave 0 --sim-address=127.0.0.1 \
    --sysid "$sysid" -I "$instance" \
    > "/tmp/arducopter_${instance}.log" 2>&1 &

  sleep 3

  # Combined-view port (14599) matches multi_uav/sim/launch_swarm_sitl.sh's
  # native MAP_PORT convention — all drones forward there, host tells them
  # apart by sysid.
  mavproxy.py --master "tcp:127.0.0.1:${tcp_port}" \
    --out "udp:127.0.0.1:${out_port}" \
    --out "udp:${GATEWAY_IP}:14599" \
    --non-interactive \
    > "/tmp/mavproxy_${instance}.log" 2>&1 &
}

if [[ -n "$SWARM_DRONES" ]]; then
  # Multi-UAV: SWARM_DRONES=2 or 3. On the HOST, watch all of them with:
  #   mavproxy.py --master udp:0.0.0.0:14599 --console --map
  echo "[entrypoint] Starting ${SWARM_DRONES} SITL instance(s) for multi-UAV …"
  launch_drone 0 1 14550
  [[ "$SWARM_DRONES" -ge 2 ]] && launch_drone 1 2 14560
  [[ "$SWARM_DRONES" -ge 3 ]] && launch_drone 2 3 14570
else
  # Single-drone (default). On the HOST, watch it with:
  #   mavproxy.py --master udp:0.0.0.0:14599 --console --map
  launch_drone 0 1 14550
fi

# Give SITL a head start; main.py/swarm_main.py themselves retry on GPS/EKF
# readiness anyway.
sleep 15

cd /app
exec "$@"
