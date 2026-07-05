# Omokai UAV Pipeline

**Prompt → LLM → Validated Mission JSON → Deterministic Executor → ArduPilot SITL**

End-to-end autonomous drone pipeline: give a natural-language command, watch the drone fly.

## Architecture

```
Operator Prompt (natural language)
          │
          ▼
   ┌─────────────┐
   │ LLM Planner │  DeepSeek (aicredits.in) — interprets intent, emits JSON.
   │(single_drone/│  The LLM NEVER touches the vehicle. It proposes only.
   │ src/llm_    │
   │ planner.py) │
   └──────┬──────┘
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
   │ArduPilot SITL          │  Full ArduCopter flight stack running in
   │+ Gazebo Harmonic (gz)  │  simulation against a Gazebo Harmonic world.
   └────────────────────────┘
```

### Key design guarantees

| Guarantee | How it's enforced |
|---|---|
| LLM never in control loop | `llm_planner.py` is never imported by `mission_executor.py` |
| Same JSON = same flight | Executor is pure function: JSON → MAVLink sequence |
| Mission auditable | Full JSON written to disk before execution |
| Mission blocked if unsafe | Validator rejects before executor is called |

---

## Stack

| Component | Choice | Why |
|---|---|---|
| Simulator | ArduPilot SITL + Gazebo Harmonic | Industry-standard flight stack; real ArduCopter code runs in-the-loop |
| Vehicle control | pymavlink | Lightweight, no ROS dependency for the core path |
| LLM | DeepSeek (aicredits.in, OpenAI-compatible) | Fast, cheap, follows structured output reliably |
| Validation | jsonschema + custom safety rules | Deterministic; schema-first design |
| Vision (Challenge 3) | YOLOv8 (ultralytics) + OpenCV | State-of-the-art real-time detection |

---

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

### 3. Start the simulator (Terminal 1)

```bash
# Makes sure gz_ws is built and starts Gazebo + SITL
./single_drone/sim/launch_sitl.sh
```

Wait until you see `ArduCopter` and `EKF3 IMU0 is using GPS` in the SITL console.

### 4. Run the demo (Terminal 2)

```bash
cd single_drone

# Interactive mode
python3 main.py

# Or pass a prompt directly
python3 main.py --prompt "Patrol the perimeter loop twice at 15 metres"

# Dry run — plan + validate without flying
python3 main.py --prompt "Hover at 20 metres for 10 seconds then return" --dry-run

# Load a pre-built mission JSON (no API key needed)
python3 main.py --load missions/examples/patrol_loop.json --auto-arm
```

### 5. Example prompts

```
"Patrol the perimeter loop twice at 15 metres"
"Do a grid survey at 30 metres then come back"
"Fly a 200m square at 25 metres and return to base"
"Loiter at 10 metres for 30 seconds then land"
"Inspect the area — three slow passes at 20 metres"
```

---

## Multi-UAV Quick Start

### 1. Start N SITL instances (Terminal 1)

```bash
# Launches a tmux session with 2 or 3 ArduCopter SITL instances + a combined map
./multi_uav/sim/launch_swarm_sitl.sh 3
```

Wait for `EKF3 IMU0 is using GPS` in all vehicle console panes.

### 2. Run the swarm demo (Terminal 2, or the tmux pane it opens)

```bash
cd multi_uav

# Interactive mode
python3 swarm_main.py

# Or pass a prompt directly
python3 swarm_main.py --prompt "Fly 3 drones in wedge formation at 20 metres"

# Dry run — plan, validate, and print the formation without connecting
python3 swarm_main.py --load missions/examples/wedge_3drones.json --dry-run

# Load a pre-built swarm mission JSON (no API key needed)
python3 swarm_main.py --load missions/examples/wedge_3drones.json --auto-arm
python3 swarm_main.py --load missions/examples/line_2drones.json --auto-arm
```

---

## Docker (portable, examiner machine)

```bash
# Build
docker build -t omokai-uav .

# Run with your API key (requires X11 for Gazebo display)
docker run -e LLM_API_KEY=$LLM_API_KEY \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           omokai-uav \
           python3 single_drone/main.py --prompt "Patrol the perimeter twice at 15 metres" --auto-arm
```

For headless (no display):
```bash
docker run -e LLM_API_KEY=$LLM_API_KEY \
           omokai-uav \
           python3 single_drone/main.py --load single_drone/missions/examples/patrol_loop.json --auto-arm
```

---

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

---

## Challenge 3 — Vision AI Target Detection + Follow

```bash
cd single_drone

# With vision enabled (detects "person" by default)
python3 main.py --prompt "Patrol at 15m" --vision person

# Detect cars
python3 main.py --load missions/examples/patrol_loop.json --vision car --auto-arm
```

**How it works:**

1. `TargetDetector` (`single_drone/src/vision/detector.py`) runs YOLOv8n on the camera feed.
2. On detection of the configured class:
   - Saves a timestamped JPEG snapshot to `snapshots/`
   - Prints an alert to the operator console
   - Fires the `on_detect` callback
3. `TargetFollower` (`single_drone/src/vision/follower.py`) receives the callback:
   - Switches the drone to GUIDED mode
   - Issues velocity setpoints (proportional control on pixel error → yaw rate)
   - Returns to LOITER if target lost for > 3 seconds

**Camera source:** defaults to OpenCV camera 0. For the full Gazebo setup,
replace with the GStreamer pipeline:
```python
detector = TargetDetector(
    target_class="person",
    camera_source="udpsrc port=5600 ! application/x-rtp,encoding-name=H264 ! "
                  "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)
```

**Install vision dependencies:**
```bash
pip3 install ultralytics opencv-python
```

---

## Challenge 1 — Multi-Agent Formation

**Implemented in `multi_uav/`** (mirrors the single-drone pipeline, one stage per file):

1. **Swarm JSON**: `config/swarm_mission_schema.json` extends the mission schema with a `formation` block (type, spacing, leader) and a `vehicles` list.
2. **LLM Swarm Planner** (`src/swarm_llm_planner.py`): prompts the LLM with formation geometry → emits one swarm mission JSON (leader route + formation metadata).
3. **Swarm Validator** (`src/swarm_validator.py`): formation-vs-vehicle-count checks, leader/vehicle-ID whitelist, and the same altitude/speed/geofence rules as single-drone (reuses `single_drone/src/mission_validator.py`).
4. **Formation Planner** (`src/formation_planner.py`): computes each follower's per-drone mission as `leader_pos + formation_offset` in the NED frame (line/column/wedge/triangle/diamond/custom).
5. **Swarm Executor** (`src/swarm_executor.py`): drives N `MissionExecutor` instances (reused from `single_drone/src/mission_executor.py`) in lockstep across threads, synchronized with `threading.Barrier` at each flight phase (connect → arm → takeoff → mission → RTL).

Entry point: `multi_uav/swarm_main.py`. Tests: `multi_uav/swarm_validate.py`. See [Multi-UAV Quick Start](#multi-uav-quick-start) below.

---

## Challenge 2 — SLAM / Autonomous Navigation (Architecture)

**Approach:**

1. **SLAM**: Use SLAM Toolbox (ROS 2 Humble) with a simulated 2D lidar in Gazebo.
2. **Navigation**: Replace fixed waypoints with Nav2 goal poses. The LLM emits `{x, y, frame}` goals instead of `{lat, lon}`. The executor calls the Nav2 `NavigateToPose` action server.
3. **Unknown environment**: Drone explores using a frontier-exploration node. The LLM specifies the search area; the planner decides the path dynamically based on the live map.
4. **Integration point**: The `MissionExecutor` gets a `nav2` flag in the JSON. If set, it publishes to the Nav2 action server instead of the MAVLink mission upload path.

---

## Scaling to Real-World Systems

| Demo limitation | Production fix |
|---|---|
| Single agent, SITL | Multi-vehicle with real autopilots; formation controller on companion computer |
| Claude API (cloud) | Local LLM (Llama 3 / Mistral) for air-gapped ops; Claude as fallback |
| Simple geofence box | 3D airspace polygon, NOTAM integration, weather-aware ceiling |
| No operator confirmation loop | Web UI with mission preview map, one-click approve/reject |
| pymavlink direct | ROS 2 MAVROS for richer sensor fusion and multi-robot middleware |
| YOLOv8n (nano) | YOLOv8x or RT-DETR on Jetson Orin; send compressed JPEG over 4G |
| No redundancy | Watchdog heartbeat; automatic RTL on comms loss |

---

## Sources & Licenses

| Source | License | What was taken |
|---|---|---|
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | GPL-3.0 | SITL infrastructure, MAVLink protocol |
| [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) | GPL-3.0 | Gazebo Harmonic plugin, iris world |
| [pymavlink](https://github.com/ArduPilot/pymavlink) | LGPL-3.0 | MAVLink Python bindings |
| [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) | AGPL-3.0 | Detection model + inference API |
| [jsonschema](https://github.com/python-jsonschema/jsonschema) | MIT | JSON Schema validation |
| [Anthropic Python SDK](https://github.com/anthropics/anthropic-sdk-python) | MIT | Claude API client |
| [ChatDrones](https://github.com/Gaurang-1402/ChatDrones) | MIT | Architecture reference for NL→drone pipeline |
| [MAVLink-AI-Agent](https://github.com/SuperMK15/MAVLink-AI-Agent) | MIT | Reference for LLM→MAVLink architecture |

All application code (`single_drone/`, `multi_uav/`) is original work.
