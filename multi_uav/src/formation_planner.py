import copy
import math
import uuid
from datetime import datetime, timezone
from typing import List, Tuple

_LINE = {
    # Side by side, perpendicular to direction of travel
    2: [(0.0,  0.0), (0.0,  1.0)],
    3: [(0.0,  0.0), (0.0,  1.0), (0.0, -1.0)],
}

_COLUMN = {
    # Single file, one behind the other
    2: [(0.0, 0.0), (-1.0, 0.0)],
    3: [(0.0, 0.0), (-1.0, 0.0), (-2.0, 0.0)],
}

_WEDGE = {
    # V-shape: leader at front tip, followers behind and to the sides
    2: [(0.0, 0.0), (-1.0,  1.0)],
    3: [(0.0, 0.0), (-1.0,  1.0), (-1.0, -1.0)],
}

_TRIANGLE = {
    # Equilateral triangle: leader at front vertex
    3: [(0.0, 0.0), (-1.0,  0.866), (-1.0, -0.866)],
}

_DIAMOND = {
    # Leader at front, followers flanking at mid-rear
    3: [(0.0, 0.0), (-0.5,  1.0), (-0.5, -1.0)],
}

_LEADER_FOLLOWER = {
    # Each vehicle directly behind the one ahead
    2: [(0.0, 0.0), (-1.0, 0.0)],
    3: [(0.0, 0.0), (-1.0, 0.0), (-2.0, 0.0)],
}

_FORMATION_TABLE = {
    "line":            _LINE,
    "column":          _COLUMN,
    "wedge":           _WEDGE,
    "triangle":        _TRIANGLE,
    "diamond":         _DIAMOND,
    "leader_follower": _LEADER_FOLLOWER,
}

def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Forward azimuth in degrees [0, 360) from point 1 to point 2.
    0° = North, 90° = East, 180° = South, 270° = West.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(phi2)
    y = (math.cos(phi1) * math.sin(phi2)
         - math.sin(phi1) * math.cos(phi2) * math.cos(dl))
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _apply_ned_offset(
    lat: float,
    lon: float,
    bearing_deg: float,
    fwd_m: float,
    right_m: float,
) -> Tuple[float, float]:
    """
    Apply a (fwd_m, right_m) body-frame offset to (lat, lon), where the
    body frame is oriented at bearing_deg from North.

    NED derivation:
        forward unit vector: N = cos(b), E = sin(b)
        right unit vector:   N = sin(b+90°) = -sin(b),  E = cos(b+90°) = ... wait
        right (90° CW from forward):
            right_N = cos(b + 90°) = -sin(b)
            right_E = sin(b + 90°) =  cos(b)   -- no, let me do it properly

        Heading East (b=90°): forward=(0,1), right should be South=(-1,0)
            right_N = -sin(90°) = -1  ✓
            right_E =  cos(90°) =  0  ✓

        Heading North (b=0°): forward=(1,0), right should be East=(0,1)
            right_N = -sin(0°) = 0  ✓
            right_E =  cos(0°) = 1  ✓

        delta_N = fwd_m * cos(b) + right_m * (-sin(b))
        delta_E = fwd_m * sin(b) + right_m *   cos(b)
    """
    b = math.radians(bearing_deg)
    cos_b = math.cos(b)
    sin_b = math.sin(b)

    delta_n = fwd_m * cos_b - right_m * sin_b
    delta_e = fwd_m * sin_b + right_m * cos_b

    lat_per_m = 1.0 / 111111.0
    lon_per_m = 1.0 / (111111.0 * math.cos(math.radians(lat)))

    return lat + delta_n * lat_per_m, lon + delta_e * lon_per_m


def _leg_bearings(waypoints: List[dict]) -> List[float]:
    n = len(waypoints)
    if n == 1:
        return [0.0]
    bearings = [
        _bearing(
            waypoints[i]["lat"], waypoints[i]["lon"],
            waypoints[i + 1]["lat"], waypoints[i + 1]["lon"],
        )
        for i in range(n - 1)
    ]
    bearings.append(bearings[-1])
    return bearings


def _offset_waypoints(
    waypoints: List[dict],
    bearings: List[float],
    fwd_m: float,
    right_m: float,
) -> List[dict]:
    result = []
    for wp, brg in zip(waypoints, bearings):
        new_lat, new_lon = _apply_ned_offset(
            wp["lat"], wp["lon"], brg, fwd_m, right_m
        )
        new_wp = copy.deepcopy(wp)
        new_wp["lat"] = round(new_lat, 7)
        new_wp["lon"] = round(new_lon, 7)
        result.append(new_wp)
    return result


def _build_vehicle_mission(
    swarm_mission: dict,
    vehicle_id: str,
    waypoints: List[dict],
) -> dict:
    """Construct a single-vehicle mission dict compatible with MissionExecutor."""
    return {
        "mission_id":            str(uuid.uuid4()),
        "created_at":            datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "natural_language_input": swarm_mission.get("natural_language_input", ""),
        "vehicle_id":            vehicle_id,
        "home_location":         swarm_mission["home_location"],
        "parameters":            swarm_mission["parameters"],
        "waypoints":             waypoints,
    }

class FormationPlanner:
    """
    Takes a validated swarm mission dict and returns N per-drone mission dicts.
    Each output mission is compatible with MissionExecutor (same schema).
    No AI, no external calls — pure geometry.
    """

    def plan(self, swarm_mission: dict) -> List[dict]:
        """
        Returns one mission dict per vehicle, in the same order as
        swarm_mission['vehicles'].
        """
        formation  = swarm_mission["formation"]
        f_type     = formation["type"]
        spacing_m  = float(formation["spacing_m"])
        leader_id  = formation["leader_id"]
        vehicles   = swarm_mission["vehicles"]
        leader_wps = swarm_mission["waypoints"]
        overrides  = swarm_mission.get("vehicle_overrides", {})

        bearings = _leg_bearings(leader_wps)

        missions = []
        for vehicle_id in vehicles:
            if vehicle_id in overrides:
                # Per-vehicle route override takes full precedence
                wps = copy.deepcopy(overrides[vehicle_id]["waypoints"])
            else:
                fwd_m, right_m = self._vehicle_offset(
                    f_type, spacing_m, vehicles, leader_id, vehicle_id, formation
                )
                if fwd_m == 0.0 and right_m == 0.0:
                    wps = copy.deepcopy(leader_wps)
                else:
                    wps = _offset_waypoints(leader_wps, bearings, fwd_m, right_m)

            missions.append(_build_vehicle_mission(swarm_mission, vehicle_id, wps))

        return missions

    def _vehicle_offset(
        self,
        f_type: str,
        spacing_m: float,
        vehicles: List[str],
        leader_id: str,
        vehicle_id: str,
        formation: dict,
    ) -> Tuple[float, float]:
        """Return the (fwd_m, right_m) world offset for this vehicle."""
        if vehicle_id == leader_id:
            return (0.0, 0.0)

        if f_type == "custom":
            co = formation.get("custom_offsets", {}).get(vehicle_id, {})
            return (float(co.get("fwd_m", 0.0)), float(co.get("right_m", 0.0)))

        n = len(vehicles)
        table = _FORMATION_TABLE.get(f_type)
        if table is None:
            raise ValueError(f"Unknown formation type: '{f_type}'")
        raw = table.get(n)
        if raw is None:
            raise ValueError(
                f"Formation '{f_type}' has no offset definition for {n} vehicles."
            )

        # raw[0] = leader slot, raw[1..] = follower slots in order
        followers = [v for v in vehicles if v != leader_id]
        follower_idx = followers.index(vehicle_id)
        fwd_mult, right_mult = raw[1 + follower_idx]
        return (fwd_mult * spacing_m, right_mult * spacing_m)


def plan_formation(swarm_mission: dict) -> List[dict]:
    """Convenience wrapper — returns per-drone mission list."""
    return FormationPlanner().plan(swarm_mission)
