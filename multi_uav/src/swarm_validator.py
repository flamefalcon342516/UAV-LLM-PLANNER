"""
Swarm Mission Validator — guardrail between the LLM and the swarm executor.

Validation pipeline:
    1. JSON Schema  (config/swarm_mission_schema.json)
    2. Formation compatibility  (type vs vehicle count)
    3. Vehicle ID whitelist and leader presence
    4. Safety limits  (altitude, speed, loops — same rules as single-drone)
    5. Geofence  (all leader waypoints inside bounding box)
    6. Waypoint geometry  (spacing, duplicates, altitude per-WP)

The executor never receives an invalid swarm mission.
"""

import json
import math
from pathlib import Path
from typing import List
import jsonschema
import yaml

from single_drone.src.mission_validator import haversine

_CONFIG_DIR = Path(__file__).parent.parent / "config"
_SWARM_SCHEMA_PATH = _CONFIG_DIR / "swarm_mission_schema.json"
_SAFETY_PATH = _CONFIG_DIR / "safety_config.yaml"

_FORMATION_COUNTS = {
    "line":            {2, 3},
    "column":          {2, 3},
    "wedge":           {2, 3},
    "triangle":        {3},
    "diamond":         {3},
    "leader_follower": {2, 3},
    "custom":          {2, 3},
}
_KNOWN_VEHICLE_IDS = {"copter_1", "copter_2", "copter_3"}


def _load_swarm_schema() -> dict:
    with open(_SWARM_SCHEMA_PATH) as f:
        return json.load(f)


def _load_safety() -> dict:
    with open(_SAFETY_PATH) as f:
        return yaml.safe_load(f)


class SwarmValidationError(Exception):
    pass


class SwarmMissionValidator:

    def __init__(self):
        self.schema = _load_swarm_schema()
        self.safety = _load_safety()

    def validate(self, mission: dict) -> dict:
        errors: List[str] = []

        errors.extend(self._check_schema(mission))
        if errors:
            raise SwarmValidationError(
                "\n".join(["Swarm mission rejected:"] + [f"  • {e}" for e in errors])
            )
        errors.extend(self._check_formation(mission))
        errors.extend(self._check_vehicles(mission))
        errors.extend(self._check_safety(mission))
        errors.extend(self._check_geometry(mission))

        if errors:
            raise SwarmValidationError(
                "\n".join(["Swarm mission rejected:"] + [f"  • {e}" for e in errors])
            )

        return mission

    def _check_schema(self, mission: dict) -> List[str]:
        validator = jsonschema.Draft7Validator(self.schema)
        return [e.message for e in validator.iter_errors(mission)]


    def _check_formation(self, mission: dict) -> List[str]:
        errors: List[str] = []
        formation = mission.get("formation", {})
        f_type = formation.get("type", "")
        vehicles = mission.get("vehicles", [])
        n = len(vehicles)

        allowed = _FORMATION_COUNTS.get(f_type, set())
        if n not in allowed:
            errors.append(
                f"Formation '{f_type}' requires {sorted(allowed)} vehicle(s); "
                f"{n} specified."
            )

        leader_id = formation.get("leader_id", "")
        if leader_id not in vehicles:
            errors.append(
                f"leader_id '{leader_id}' is not in vehicles list {vehicles}."
            )

        if f_type == "custom":
            custom = formation.get("custom_offsets", {})
            for vid in vehicles:
                if vid == leader_id:
                    continue
                if vid not in custom:
                    errors.append(
                        f"Formation 'custom' requires a custom_offsets entry "
                        f"for follower '{vid}'."
                    )

        return errors

    def _check_vehicles(self, mission: dict) -> List[str]:
        errors: List[str] = []
        vehicles = mission.get("vehicles", [])

        for vid in vehicles:
            if vid not in _KNOWN_VEHICLE_IDS:
                errors.append(
                    f"Unknown vehicle ID '{vid}'. "
                    f"Allowed: {sorted(_KNOWN_VEHICLE_IDS)}."
                )

        seen: set = set()
        for vid in vehicles:
            if vid in seen:
                errors.append(f"Duplicate vehicle ID '{vid}'.")
            seen.add(vid)

        return errors

    def _check_safety(self, mission: dict) -> List[str]:
        errors: List[str] = []
        s = self.safety
        params = mission.get("parameters", {})

        alt = params.get("altitude_m", 0)
        if alt < s["altitude"]["min_m"]:
            errors.append(f"Altitude {alt} m is below minimum {s['altitude']['min_m']} m.")
        if alt > s["altitude"]["max_m"]:
            errors.append(f"Altitude {alt} m exceeds ceiling {s['altitude']['max_m']} m.")

        speed = params.get("groundspeed_ms", 0)
        if speed < s["speed"]["min_groundspeed_ms"]:
            errors.append("Ground speed is below minimum.")
        if speed > s["speed"]["max_groundspeed_ms"]:
            errors.append("Ground speed exceeds maximum.")

        loops = params.get("loops", 1)
        if loops < 1:
            errors.append("Loop count must be at least 1.")
        if loops > s["mission"]["max_loops"]:
            errors.append(
                f"Loop count {loops} exceeds maximum {s['mission']['max_loops']}."
            )

        waypoints = mission.get("waypoints", [])
        max_wp = s["mission"].get("max_waypoints", 50)
        if len(waypoints) > max_wp:
            errors.append(
                f"Mission has {len(waypoints)} waypoints (maximum {max_wp})."
            )

        for wp in waypoints:
            wp_alt = wp.get("alt_m", 0)
            if wp_alt < s["altitude"]["min_m"]:
                errors.append(f"Waypoint {wp['id']} is below minimum altitude.")
            if wp_alt > s["altitude"]["max_m"]:
                errors.append(f"Waypoint {wp['id']} exceeds altitude ceiling.")

        return errors

    def _check_geometry(self, mission: dict) -> List[str]:
        errors: List[str] = []
        s = self.safety
        waypoints = mission.get("waypoints", [])

        if s["geofence"]["enabled"]:
            fence = s["geofence"]
            for wp in waypoints:
                if not (fence["lat_min"] <= wp["lat"] <= fence["lat_max"]):
                    errors.append(
                        f"Waypoint {wp['id']} is outside latitude geofence."
                    )
                if not (fence["lon_min"] <= wp["lon"] <= fence["lon_max"]):
                    errors.append(
                        f"Waypoint {wp['id']} is outside longitude geofence."
                    )

        # Duplicate coordinates
        seen: set = set()
        for wp in waypoints:
            key = (round(wp["lat"], 7), round(wp["lon"], 7), round(wp["alt_m"], 2))
            if key in seen:
                errors.append(
                    f"Duplicate waypoint coordinates at id={wp['id']}."
                )
            seen.add(key)

        # Adjacent waypoint spacing
        min_sp = s["mission"].get("min_wp_spacing_m", 2)
        max_sp = s["mission"].get("max_wp_spacing_m", 500)
        for i in range(len(waypoints) - 1):
            a, b = waypoints[i], waypoints[i + 1]
            d = haversine(a["lat"], a["lon"], b["lat"], b["lon"])
            if d < min_sp:
                errors.append(
                    f"Waypoints {a['id']} and {b['id']} are only {d:.1f} m apart."
                )
            if d > max_sp:
                errors.append(
                    f"Waypoints {a['id']} and {b['id']} are {d:.1f} m apart (too far)."
                )

        # Home location present
        home = mission.get("home_location", {})
        if "lat" not in home or "lon" not in home:
            errors.append("Mission is missing home_location.")

        return errors


def validate_swarm_mission(mission: dict) -> dict:
    return SwarmMissionValidator().validate(mission)
