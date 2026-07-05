# Omokai UAV Pipeline

**Prompt → LLM → Validated Mission JSON → Deterministic Executor → ArduPilot SITL**

End-to-end autonomous drone pipeline: give a natural-language command, watch the drone fly.

## Architecture

```
Operator Prompt (natural language)
          │
          ▼
   ┌───────────────┐
   │ LLM Planner   │  DeepSeek (aicredits.in) — interprets intent, emits JSON.
   │(single_drone/ │  The LLM NEVER touches the vehicle. It proposes only.
   │ src/llm_      │
   │ planner.py)   │
   └──────┬────────┘
          │  Mission JSON (uuid, waypoints, alt, speed, loops…)
          ▼
   ┌──────────────────────┐
   │ Mission Validator    │  JSON Schema + safety rules (altitude ceiling,
   │(single_drone/src/    │  speed limits, geofence bounding box).
   │ mission_validator.py)│  Rejects the mission entirely if any rule fails.
   └────────┬─────────────┘
            │  Validated JSON only
            ▼
   ┌──────────────────────┐
   │Deterministic Executor│  Reads validated JSON → MAVLink commands.
   │(single_drone/src/    │  Same JSON = same flight every time.
   │ mission_executor.py) │  No LLM calls here. Fully auditable.
   └────────┬─────────────┘
            │  MAVLink (UDP 14550)
            ▼
   ┌────────────────────────┐
   │     ArduPilot SITL     │  Full ArduCopter flight stack running in  
   └────────────────────────┘
```

## Quick Start (native, Ubuntu 22.04)

### 1. Install dependencies

```bash
cd ~/omokai-uav
pip3 install -r requirements.txt
```

### 2. Set API key

```bash
export LLM_API_KEY="sk-live-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```
### Will be shared on personal mail, it's my purchased Deepseek-v4-pro/API_KEY

### 3. Start the simulator (Terminal 1)

```bash
./single_drone/sim/launch_sitl.sh
```
Wait until you see `ArduCopter` and `EKF3 IMU0 is using GPS` in the SITL console.
Helps prevent Arm and takeoff cmd failures.!!

### 4. Run the demo (Terminal 2)

```bash
cd single_drone
python3 main.py
python3 main.py --prompt "Patrol the perimeter loop twice at 15 metres"
python3 main.py --prompt "Patrol the perimeter loop twice at 15 metres" --auto-arm
python3 main.py --load missions/examples/patrol_loop.json --auto-arm         # the example json used for testing
```

### 5. Example prompts (All verified,, sometime it might throw the LLM Timeout error, thats bcs of the slow API endpoint)

```
"Patrol the perimeter loop twice at 15 metres"
"Do a grid survey at 30 metres then come back"
"Fly a 200m square at 25 metres and return to base"
"Loiter at 10 metres for 30 seconds then land"
"Inspect the area — three slow passes at 20 metres"
```

## Multi-UAV Quick Start

### 1. Start N SITL instances (Terminal 1)

```bash
# Launches a tmux session with 3 ArduCopter SITL instances + a combined map (if 3, not mentioned default 3 drones spawn)
./multi_uav/sim/launch_swarm_sitl.sh 3
```
Wait for `25 sec`, for the combined mavproxy instance to open , which has all the three drones as updin
Wait for `EKF3 IMU0 is using GPS` in all vehicle console panes.

### 2. Run the swarm demo (Terminal 2, or the tmux pane it opens)

```bash
cd multi_uav
python3 swarm_main.py
python3 swarm_main.py --prompt "Fly 3 drones in wedge formation at 20 metres"
python3 swarm_main.py --prompt "Fly 3 drones in wedge formation at 20 metres" --auto-arm
# Load a pre-built swarm mission JSON (no API key needed)
python3 swarm_main.py --load missions/examples/wedge_3drones.json --auto-arm
python3 swarm_main.py --load missions/examples/line_2drones.json --auto-arm
```

## Docker (portable, examiner machine)

Ubuntu 22.04 + ArduPilot SITL only 
single-drone and multi-UAV (2 or 3 vehicles). Both flown end-to-end in testing.

```bash
# Build
docker build -t omokai-uav .
```

Note: SITL and MAVProxy are launched directly by `docker-entrypoint.sh`,
bypassing ArduPilot's own `sim_vehicle.py` wrapper — its process supervision
was found to kill MAVProxy within seconds inside a container, every time,
regardless of network/timing conditions. Launching the two processes directly
instead runs indefinitely with no issue.

### Single-drone

Always use `-it` — without it Python's stdout is fully buffered inside Docker
and you won't see anything until the process exits.

```bash
# Pre-built mission (no API key needed)
docker run -it --rm omokai-uav \
           python3 single_drone/main.py --load single_drone/missions/examples/patrol_loop.json --auto-arm

# LLM prompt (API Key to be shared via mail)
docker run -it --rm -e LLM_API_KEY=$LLM_API_KEY omokai-uav \
           python3 single_drone/main.py --prompt "Patrol the perimeter loop twice at 15 metres" --auto-arm
```

### Multi-UAV

Set `SWARM_DRONES=2` or `3` to start that many SITL instances instead of one:

```bash
docker run -it -e SWARM_DRONES=3 -e LLM_API_KEY=$LLM_API_KEY omokai-uav bash

# once inside the container:
cd multi_uav
python3 swarm_main.py --load missions/examples/wedge_3drones.json --auto-arm
```

### Watching it live (map + console)

The container proactively forwards telemetry to the Docker bridge gateway, so
no `-p` port publishing is needed — just run this on your **host** (using your
native `mavproxy.py`) while a container is running:

```bash
mavproxy.py --master udp:0.0.0.0:14599 --console --map
```

Single-drone or multi-UAV, all vehicles show up on this one connection
(distinguished by sysid) — it's view-only and never touches the actual
mission-control connections `main.py`/`swarm_main.py` use internally.


## Mission JSON Format

Every mission that reaches the executor must match this schema (enforced by jsonschema):

```json
{
  "mission_id": "uuid-v4",
  "created_at": "ISO-8601 timestamp",
  "natural_language_input": "original operator prompt",
  "vehicle_id": "copter_1",
  "home_location": { "lat": -35.363261, "lon": 149.165230, "alt_m": 0 },
  "parameters": {
    "altitude_m": 15,          // 2–120 m (hard limit)
    "groundspeed_ms": 5,       // 0.5–20 m/s (hard limit)
    "loops": 2,                // 1–10
    "return_to_home": true,
    "loiter_time_s": 0
  },
  "waypoints": [
    { "id": 1, "lat": -35.362361, "lon": 149.164130, "alt_m": 15,
      "action": "none", "hold_s": 0 }
  ]
}
```

Safety rules enforced by `MissionValidator` before any execution:
- `altitude_m` must be 2–120 m
- `groundspeed_ms` must be 0.5–20 m/s
- All waypoints must fall within the geofence bounding box
- Schema violations (wrong types, missing fields) are rejected

## Challenge 1 — Multi-Agent Formation

**Implemented in `multi_uav/`** (mirrors the single-drone pipeline, one stage per file):

1. **Swarm JSON**: `config/swarm_mission_schema.json` extends the mission schema with a `formation` block (type, spacing, leader) and a `vehicles` list.
2. **LLM Swarm Planner** (`src/swarm_llm_planner.py`): prompts the LLM with formation geometry → emits one swarm mission JSON (leader route + formation metadata).
3. **Swarm Validator** (`src/swarm_validator.py`): formation-vs-vehicle-count checks, leader/vehicle-ID whitelist, and the same altitude/speed/geofence rules as single-drone (reuses `single_drone/src/mission_validator.py`).
4. **Formation Planner** (`src/formation_planner.py`): computes each follower's per-drone mission as `leader_pos + formation_offset` in the NED frame (line/column/wedge/triangle/diamond/custom).
5. **Swarm Executor** (`src/swarm_executor.py`): drives N `MissionExecutor` instances (reused from `single_drone/src/mission_executor.py`) in lockstep across threads, synchronized with `threading.Barrier` at each flight phase (connect → arm → takeoff → mission → RTL).

Entry point: `multi_uav/swarm_main.py`. Tests: `multi_uav/swarm_validate.py`. See [Multi-UAV Quick Start](#multi-uav-quick-start) below.

## Sources & Licenses

| Source | License | What was taken |
|---|---|---|
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | GPL-3.0 | SITL infrastructure, MAVLink protocol |
| [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) | GPL-3.0 | Gazebo Harmonic plugin, iris world |
| [pymavlink](https://github.com/ArduPilot/pymavlink) | LGPL-3.0 | MAVLink Python bindings |
| [jsonschema](https://github.com/python-jsonschema/jsonschema) | MIT | JSON Schema validation |
| [Anthropic Python SDK](https://github.com/anthropics/anthropic-sdk-python) | MIT | Claude API client |
| [ChatDrones](https://github.com/Gaurang-1402/ChatDrones) | MIT | Architecture reference for NL→drone pipeline |
| [MAVLink-AI-Agent](https://github.com/SuperMK15/MAVLink-AI-Agent) | MIT | Reference for LLM→MAVLink architecture |

All application code (`single_drone/`, `multi_uav/`) is original work.
