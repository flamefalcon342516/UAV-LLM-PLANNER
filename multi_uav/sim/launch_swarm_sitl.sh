#!/usr/bin/env bash
# Omokai UAV — Swarm SITL tmux launcher
# Usage:
#   ./sim/launch_swarm_sitl.sh        # 3 drones (default)
#   ./sim/launch_swarm_sitl.sh 2      # 2 drones
#
# How the combined map works:
#   Each SITL forwards to its own port (14550/60/70) for swarm_main.py
#   AND to a shared port (14599) for the combined map mavproxy.
#   MAVProxy on 14599 receives all 3 streams, distinguishes by sysid,
set -e

DRONES="${1:-3}"
ARDUPILOT_DIR="${HOME}/ardupilot"
PROJECT_DIR="${HOME}/omokai-uav/multi_uav"
SESSION="omokai-swarm"

if [[ ! -f "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" ]]; then
    echo "ERROR: ArduPilot not found at ${ARDUPILOT_DIR}"
    exit 1
fi

if [[ "$DRONES" -ne 2 && "$DRONES" -ne 3 ]]; then
    echo "Usage: $0 [2|3]"
    exit 1
fi

if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux is not installed. Install with: sudo apt install tmux"
    exit 1
fi

if ! command -v mavproxy.py &>/dev/null; then
    echo "WARNING: mavproxy.py not found — combined map pane will fail."
    echo "         Install with: pip3 install MAVProxy"
fi

# precautious kill
tmux kill-session -t "$SESSION" 2>/dev/null || true

# this is the sahred port
MAP_PORT=14599

#sitl
SITL="cd ${ARDUPILOT_DIR} && Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild"

#Create tmux session 
tmux new-session -d -s "$SESSION" -x 220 -y 52

# ── Build layout ────────────────────────────────────────────────────────────
#
# Start: 1 pane (full screen)
# Step 1: Split horizontally → left (consoles) | right (map + demo)
# Step 2: Split right pane vertically → top-right (map) | bot-right (demo)
# Step 3: Split left pane vertically → top-left | bot (67% remaining)
# Step 4: Split bot-left vertically → mid-left | bot-left
#
# Resulting pane indices:
#   0 → top-left    copter_1 console
#   3 → mid-left    copter_2 console
#   4 → bot-left    copter_3 console  (only created when DRONES=3)
#   1 → top-right   combined map
#   2 → bot-right   swarm_main.py

# Step 1: left 55% | right 45%
tmux split-window -h -t "$SESSION:0.0" -p 45

# Step 2: right side — map (top 68%) | demo (bot 32%)
tmux split-window -v -t "$SESSION:0.1" -p 32

# Step 3: left side — drone 1 (top 33%) | remaining (bot 67%)
tmux split-window -v -t "$SESSION:0.0" -p 67

# Step 4: split remaining left — drone 2 (top 50%) | drone 3 (bot 50%)
if [[ "$DRONES" -eq 3 ]]; then
    tmux split-window -v -t "$SESSION:0.3" -p 50
fi

# ── Name panes ──────────────────────────────────────────────────────────────
tmux select-pane -t "$SESSION:0.0" -T " copter_1 | sysid=1 | :14550 "
tmux select-pane -t "$SESSION:0.1" -T " Combined Map (all drones) "
tmux select-pane -t "$SESSION:0.2" -T " swarm_main.py "
tmux select-pane -t "$SESSION:0.3" -T " copter_2 | sysid=2 | :14560 "
if [[ "$DRONES" -eq 3 ]]; then
    tmux select-pane -t "$SESSION:0.4" -T " copter_3 | sysid=3 | :14570 "
fi

# ── Start SITL instances ────────────────────────────────────────────────────

# copter_1 — instance 0, sysid 1
# Sends to :14550 (swarm_main.py) and :MAP_PORT (combined map)
tmux send-keys -t "$SESSION:0.0" \
    "${SITL} -I 0 --sysid 1 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:${MAP_PORT} --console" \
    Enter

# copter_2 — instance 1, sysid 2
tmux send-keys -t "$SESSION:0.3" \
    "${SITL} -I 1 --sysid 2 --out udp:127.0.0.1:14560 --out udp:127.0.0.1:${MAP_PORT} --console" \
    Enter

# copter_3 — instance 2, sysid 3 (only when DRONES=3)
if [[ "$DRONES" -eq 3 ]]; then
    tmux send-keys -t "$SESSION:0.4" \
        "${SITL} -I 2 --sysid 3 --out udp:127.0.0.1:14570 --out udp:127.0.0.1:${MAP_PORT} --console" \
        Enter
fi

# ── Combined map ─────────────────────────────────────────────────────────────
# Waits 25 s for all SITL instances to boot and acquire GPS, then starts.
# Connects to the shared port (14599) where all 3 drones forward their telemetry.
# MAVProxy distinguishes vehicles by sysid (1, 2, 3) and renders all on one map.
tmux send-keys -t "$SESSION:0.1" \
    "sleep 25 && mavproxy.py --master udp:0.0.0.0:${MAP_PORT} --map --console" \
    Enter

# ── swarm_demo pane ──────────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.2" \
    "cd ${PROJECT_DIR}" \
    Enter

tmux send-keys -t "$SESSION:0.2" \
    "echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' && \
echo ' Omokai UAV — Swarm Demo Pane' && \
echo ' Wait for: EKF3 IMU0 is using GPS  in ALL 3 console panes.' && \
echo ' The map will appear automatically after ~25 seconds.' && \
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' && \
echo '' && \
echo 'Quick-start commands:' && \
echo '' && \
echo '  # Pre-built wedge mission (3 drones, no API key needed):' && \
echo '  python3 swarm_main.py --load missions/examples/wedge_3drones.json --auto-arm' && \
echo '' && \
echo '  # Pre-built line mission (2 drones):' && \
echo '  python3 swarm_main.py --load missions/examples/line_2drones.json --auto-arm' && \
echo '' && \
echo '  # LLM prompt (needs LLM_API_KEY):' && \
echo '  python3 swarm_main.py --prompt \"Fly 3 drones in wedge at 20m\" --auto-arm' && \
echo '' && \
echo '  # Dry run (no SITL needed):' && \
echo '  python3 swarm_main.py --load missions/examples/wedge_3drones.json --dry-run'" \
    Enter

# ── Focus the demo pane and attach ──────────────────────────────────────────
tmux select-pane -t "$SESSION:0.2"

echo ""
echo "┌─────────────────────────────────────────────────────┐"
echo "│        Omokai Swarm SITL — tmux session ready       │"
echo "│                                                     │"
echo "│  Session : ${SESSION}                               │"
echo "│  Drones  : ${DRONES}                                │"
echo "│                                                     │"
echo "│  Attach  : tmux attach -t ${SESSION}                │"
echo "│  Detach  : Ctrl-b  d                                │"
echo "│  Switch  : Ctrl-b  arrow keys                       │"
echo "│  Zoom    : Ctrl-b  z  (toggle pane fullscreen)      │"
echo "└─────────────────────────────────────────────────────┘"
echo ""

tmux attach-session -t "$SESSION"
