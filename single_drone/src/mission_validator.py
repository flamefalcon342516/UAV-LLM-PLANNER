"""
Mission Validator — Guardrail layer between the LLM and the executor.

Validation pipeline:
    1. JSON Schema
    2. Vehicle safety limits
    3. Geofence
    4. Mission sanity
    5. Waypoint geometry

The executor NEVER receives an invalid mission.
"""

import json
import math
from pathlib import Path

import jsonschema
import yaml

_CONFIG_DIR = Path(__file__).parent.parent / "config"
_SCHEMA_PATH = _CONFIG_DIR / "mission_schema.json"
_SAFETY_PATH = _CONFIG_DIR / "safety_config.yaml"

def _load_schema():
    with open(_SCHEMA_PATH) as f:
        return json.load(f)

def _load_safety():
    with open(_SAFETY_PATH) as f:
        return yaml.safe_load(f)

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1)
        * math.cos(phi2)
        * math.sin(dlambda / 2) ** 2
    )
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

class ValidationError(Exception):
    pass

class MissionValidator:

    def __init__(self):

        self.schema = _load_schema()
        self.safety = _load_safety()
    def validate(self, mission):

        errors = []
        errors.extend(self._check_schema(mission))
        if not errors:
            errors.extend(self._check_safety(mission))

        if errors:

            raise ValidationError(
                "\n".join(
                    ["Mission rejected:"] +
                    [f" • {e}" for e in errors]
                )
            )

        return mission

    def _check_schema(self, mission):
        validator = jsonschema.Draft7Validator(self.schema)
        return [
            e.message
            for e in validator.iter_errors(mission)
        ]

    def _check_safety(self, mission):

        errors = []

        s = self.safety

        params = mission.get("parameters", {})
        waypoints = mission.get("waypoints", [])
        home = mission.get("home_location", {})
        alt = params.get("altitude_m", 0)
        if alt < s["altitude"]["min_m"]:
            errors.append(
                f"Mission altitude {alt} m below minimum."
            )
        if alt > s["altitude"]["max_m"]:
            errors.append(
                f"Mission altitude {alt} m exceeds ceiling."
            )

        speed = params.get("groundspeed_ms", 0)
        if speed < s["speed"]["min_groundspeed_ms"]:
            errors.append(
                "Groundspeed below minimum."
            )

        if speed > s["speed"]["max_groundspeed_ms"]:
            errors.append(
                "Groundspeed exceeds maximum."
            )

        loops = params.get("loops", 1)
        if loops < 1:
            errors.append("Loop count must be at least one.")

        if loops > s["mission"]["max_loops"]:
            errors.append(
                f"Loop count {loops} exceeds maximum."
            )

        max_wp = s["mission"].get("max_waypoints", 50)

        if len(waypoints) > max_wp:

            errors.append(
                f"Mission has {len(waypoints)} waypoints "
                f"(maximum {max_wp})."
            )

        seen = set()
        last_idx = len(waypoints) - 1

        first_key = None
        if waypoints:
            first_key = (
                round(waypoints[0]["lat"], 7),
                round(waypoints[0]["lon"], 7),
                round(waypoints[0]["alt_m"], 2),
            )

        for idx, wp in enumerate(waypoints):

            key = (
                round(wp["lat"], 7),
                round(wp["lon"], 7),
                round(wp["alt_m"], 2),
            )

            # The last waypoint of a closed loop/circle is intentionally
            # identical to the first — that's not a duplicate.
            is_closing_waypoint = (
                idx == last_idx and idx > 0 and key == first_key
            )

            if key in seen and not is_closing_waypoint:

                errors.append(
                    f"Duplicate waypoint {wp['id']}."
                )

            seen.add(key)

        for wp in waypoints:

            if wp["alt_m"] < s["altitude"]["min_m"]:

                errors.append(
                    f"Waypoint {wp['id']} below minimum altitude."
                )

            if wp["alt_m"] > s["altitude"]["max_m"]:

                errors.append(
                    f"Waypoint {wp['id']} exceeds altitude ceiling."
                )

        if s["geofence"]["enabled"]:

            errors.extend(
                self._check_geofence(
                    waypoints,
                    s["geofence"],
                )
            )

        min_spacing = s["mission"].get(
            "min_wp_spacing_m",
            2,
        )

        max_spacing = s["mission"].get(
            "max_wp_spacing_m",
            500,
        )

        for i in range(len(waypoints) - 1):

            a = waypoints[i]

            b = waypoints[i + 1]

            d = haversine(
                a["lat"],
                a["lon"],
                b["lat"],
                b["lon"],
            )

            if d < min_spacing:

                errors.append(
                    f"Waypoints {a['id']} and {b['id']} "
                    f"are only {d:.1f} m apart."
                )

            if d > max_spacing:

                errors.append(
                    f"Waypoints {a['id']} and {b['id']} "
                    f"are {d:.1f} m apart."
                )

        if loops > 1 and len(waypoints) >= 2:

            first = waypoints[0]
            last = waypoints[-1]

            d = haversine(
                first["lat"],
                first["lon"],
                last["lat"],
                last["lon"],
            )

            if d > 5:

                errors.append(
                    "Loop mission does not end at the starting waypoint."
                )

        if "lat" not in home or "lon" not in home:

            errors.append(
                "Mission missing home location."
            )

        return errors

    # --------------------------------------------------------

    def _check_geofence(
        self,
        waypoints,
        fence,
    ):

        errors = []

        for wp in waypoints:

            lat = wp["lat"]
            lon = wp["lon"]

            if not (
                fence["lat_min"]
                <= lat
                <= fence["lat_max"]
            ):

                errors.append(
                    f"Waypoint {wp['id']} "
                    f"outside latitude geofence."
                )

            if not (
                fence["lon_min"]
                <= lon
                <= fence["lon_max"]
            ):

                errors.append(
                    f"Waypoint {wp['id']} "
                    f"outside longitude geofence."
                )

        return errors


# ------------------------------------------------------------

def validate_mission(mission):

    return MissionValidator().validate(mission)