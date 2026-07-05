#!/usr/bin/env python3
"""
Unit tests for the swarm pipeline stages — no SITL or network required.

Tests:
  • SwarmMissionValidator
  • FormationPlanner (offset geometry)

Run: python3 swarm_validate.py
"""

import copy
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from src.swarm_validator   import validate_swarm_mission, SwarmValidationError
from src.formation_planner import plan_formation, _bearing, _apply_ned_offset

PASS = "\033[32m✓\033[0m"
FAIL = "\033[31m✗\033[0m"
passed = failed = 0


def test(name: str, fn):
    global passed, failed
    try:
        fn()
        print(f"  {PASS} {name}")
        passed += 1
    except Exception as exc:
        print(f"  {FAIL} {name}: {exc}")
        failed += 1


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _base_wedge_3() -> dict:
    return {
        "mission_id":            "c3d4e5f6-a7b8-9012-cdef-345678901234",
        "created_at":            "2025-01-15T10:00:00Z",
        "natural_language_input": "test",
        "mission_type":          "swarm",
        "formation": {
            "type":       "wedge",
            "spacing_m":  15.0,
            "leader_id":  "copter_1",
        },
        "vehicles": ["copter_1", "copter_2", "copter_3"],
        "home_location": {"lat": -35.363261, "lon": 149.165230, "alt_m": 0},
        "parameters": {
            "altitude_m":     20,
            "groundspeed_ms": 5,
            "loops":          1,
            "return_to_home": True,
            "loiter_time_s":  0,
        },
        "waypoints": [
            {"id": 1, "lat": -35.362361, "lon": 149.164130, "alt_m": 20, "action": "none", "hold_s": 0},
            {"id": 2, "lat": -35.362361, "lon": 149.166330, "alt_m": 20, "action": "none", "hold_s": 0},
            {"id": 3, "lat": -35.364161, "lon": 149.166330, "alt_m": 20, "action": "none", "hold_s": 0},
            {"id": 4, "lat": -35.364161, "lon": 149.164130, "alt_m": 20, "action": "none", "hold_s": 0},
        ],
    }


def _base_line_2() -> dict:
    m = _base_wedge_3()
    m["mission_id"] = "d4e5f6a7-b8c9-0123-defa-456789012345"
    m["formation"]["type"] = "line"
    m["formation"]["spacing_m"] = 12.0
    m["vehicles"] = ["copter_1", "copter_2"]
    m["parameters"]["altitude_m"] = 15
    for wp in m["waypoints"]:
        wp["alt_m"] = 15
    return m


def mutate(mission: dict, **kwargs) -> dict:
    m = copy.deepcopy(mission)
    for k, v in kwargs.items():
        keys = k.split(".")
        obj = m
        for key in keys[:-1]:
            obj = obj[key]
        obj[keys[-1]] = v
    return m


# ---------------------------------------------------------------------------
# Validator tests
# ---------------------------------------------------------------------------

print("\n── SwarmMissionValidator ────────────────────────────────────────────")

test("wedge 3-drone mission is valid",
     lambda: validate_swarm_mission(_base_wedge_3()))

test("line 2-drone mission is valid",
     lambda: validate_swarm_mission(_base_line_2()))


def test_triangle_needs_3():
    m = mutate(_base_wedge_3(), **{"formation.type": "triangle"})
    m["vehicles"] = ["copter_1", "copter_2"]  # only 2
    for wp in m["waypoints"]: wp["alt_m"] = 20
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("triangle with 2 vehicles → rejected", test_triangle_needs_3)


def test_diamond_needs_3():
    m = mutate(_base_wedge_3(), **{"formation.type": "diamond"})
    m["vehicles"] = ["copter_1", "copter_2"]
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("diamond with 2 vehicles → rejected", test_diamond_needs_3)


def test_unknown_formation():
    m = mutate(_base_wedge_3(), **{"formation.type": "v_shape"})
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("unknown formation type → rejected", test_unknown_formation)


def test_leader_not_in_vehicles():
    m = copy.deepcopy(_base_wedge_3())
    m["formation"]["leader_id"] = "copter_4"
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("leader_id not in vehicles → rejected", test_leader_not_in_vehicles)


def test_unknown_vehicle_id():
    m = copy.deepcopy(_base_wedge_3())
    m["vehicles"] = ["copter_1", "copter_2", "copter_99"]
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("unknown vehicle ID → rejected", test_unknown_vehicle_id)


def test_alt_too_high():
    m = mutate(_base_wedge_3(), **{"parameters.altitude_m": 200})
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("altitude 200 m → rejected", test_alt_too_high)


def test_speed_too_high():
    m = mutate(_base_wedge_3(), **{"parameters.groundspeed_ms": 30})
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("speed 30 m/s → rejected", test_speed_too_high)



def test_waypoint_outside_geofence():
    m = copy.deepcopy(_base_wedge_3())
    m["waypoints"][0]["lat"] = -34.0   # far north of geofence
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("waypoint outside geofence → rejected", test_waypoint_outside_geofence)


def test_missing_mission_type():
    m = copy.deepcopy(_base_wedge_3())
    del m["mission_type"]
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("missing mission_type → rejected", test_missing_mission_type)


def test_wrong_mission_type():
    m = mutate(_base_wedge_3(), **{"mission_type": "single"})
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("mission_type='single' on swarm mission → rejected", test_wrong_mission_type)


def test_custom_missing_offsets():
    m = copy.deepcopy(_base_wedge_3())
    m["formation"]["type"] = "custom"
    # No custom_offsets provided
    try:
        validate_swarm_mission(m)
        raise AssertionError("Should have been rejected")
    except SwarmValidationError:
        pass
test("custom formation without custom_offsets → rejected", test_custom_missing_offsets)


# ---------------------------------------------------------------------------
# Formation planner tests
# ---------------------------------------------------------------------------

print("\n── FormationPlanner ─────────────────────────────────────────────────")

def test_wedge_3_count():
    missions = plan_formation(_base_wedge_3())
    assert len(missions) == 3, f"Expected 3, got {len(missions)}"
test("wedge 3-drone → 3 missions generated", test_wedge_3_count)


def test_line_2_count():
    missions = plan_formation(_base_line_2())
    assert len(missions) == 2, f"Expected 2, got {len(missions)}"
test("line 2-drone → 2 missions generated", test_line_2_count)


def test_vehicle_ids_preserved():
    missions = plan_formation(_base_wedge_3())
    ids = [m["vehicle_id"] for m in missions]
    assert ids == ["copter_1", "copter_2", "copter_3"], f"Got: {ids}"
test("vehicle_ids in output missions match input vehicles list", test_vehicle_ids_preserved)


def test_leader_unchanged():
    missions = plan_formation(_base_wedge_3())
    leader_out = missions[0]
    leader_src = _base_wedge_3()
    for i, wp in enumerate(leader_out["waypoints"]):
        src = leader_src["waypoints"][i]
        assert wp["lat"] == src["lat"], "Leader lat changed"
        assert wp["lon"] == src["lon"], "Leader lon changed"
test("leader waypoints unchanged by formation planner", test_leader_unchanged)


def test_wedge_follower_is_offset():
    missions = plan_formation(_base_wedge_3())
    leader_wp1   = missions[0]["waypoints"][0]
    follower_wp1 = missions[1]["waypoints"][0]
    assert leader_wp1["lat"] != follower_wp1["lat"] or \
           leader_wp1["lon"] != follower_wp1["lon"], \
        "Follower has same coords as leader (offset not applied)"
test("wedge follower waypoints are offset from leader", test_wedge_follower_is_offset)


def test_line_followers_same_lat():
    """
    In a line formation flying East, followers are displaced laterally (North/South),
    so their lat differs but the waypoint count stays the same.
    """
    missions = plan_formation(_base_line_2())
    assert len(missions[0]["waypoints"]) == len(missions[1]["waypoints"])
test("line formation produces same waypoint count per drone", test_line_followers_same_lat)


def test_column_follower_behind():
    m = copy.deepcopy(_base_wedge_3())
    m["formation"]["type"]     = "column"
    m["formation"]["spacing_m"] = 20.0
    m["vehicles"] = ["copter_1", "copter_2"]
    missions = plan_formation(m)
    # Follower's first WP should be displaced; total WP count matches
    assert len(missions[0]["waypoints"]) == len(missions[1]["waypoints"])
test("column formation produces same waypoint count per drone", test_column_follower_behind)


def test_override_used():
    m = copy.deepcopy(_base_line_2())
    override_wps = [
        {"id": 1, "lat": -35.362000, "lon": 149.165000, "alt_m": 15, "action": "none", "hold_s": 0}
    ]
    m["vehicle_overrides"] = {"copter_2": {"waypoints": override_wps}}
    missions = plan_formation(m)
    follower = next(x for x in missions if x["vehicle_id"] == "copter_2")
    assert len(follower["waypoints"]) == 1
    assert follower["waypoints"][0]["lat"] == -35.362000
test("vehicle_overrides take precedence over formation offsets", test_override_used)


def test_custom_formation():
    m = copy.deepcopy(_base_line_2())
    m["formation"]["type"] = "custom"
    m["formation"]["custom_offsets"] = {
        "copter_2": {"fwd_m": 0.0, "right_m": 25.0}
    }
    missions = plan_formation(m)
    leader   = next(x for x in missions if x["vehicle_id"] == "copter_1")
    follower = next(x for x in missions if x["vehicle_id"] == "copter_2")
    assert leader["waypoints"][0]["lat"] != follower["waypoints"][0]["lat"] or \
           leader["waypoints"][0]["lon"] != follower["waypoints"][0]["lon"]
test("custom formation applies explicit offsets", test_custom_formation)


def test_each_mission_has_parameters():
    missions = plan_formation(_base_wedge_3())
    for m in missions:
        assert "parameters"    in m
        assert "waypoints"     in m
        assert "home_location" in m
        assert "mission_id"    in m
        assert "vehicle_id"    in m
test("all per-drone missions have required fields", test_each_mission_has_parameters)


# ---------------------------------------------------------------------------
# Geometry helper tests
# ---------------------------------------------------------------------------

print("\n── Geometry helpers ─────────────────────────────────────────────────")

def test_bearing_north():
    b = _bearing(-35.363, 149.165, -35.362, 149.165)
    assert abs(b - 0.0) < 1.0, f"Expected ~0 (North), got {b:.1f}"
test("bearing to the North ≈ 0°", test_bearing_north)


def test_bearing_east():
    b = _bearing(-35.363, 149.165, -35.363, 149.166)
    assert abs(b - 90.0) < 1.0, f"Expected ~90 (East), got {b:.1f}"
test("bearing to the East ≈ 90°", test_bearing_east)


def test_bearing_south():
    b = _bearing(-35.363, 149.165, -35.364, 149.165)
    assert abs(b - 180.0) < 1.0, f"Expected ~180 (South), got {b:.1f}"
test("bearing to the South ≈ 180°", test_bearing_south)


def test_offset_north_forward():
    # Heading North, move 100 m forward → lat increases by ~0.0009
    lat, lon = _apply_ned_offset(-35.363, 149.165, 0.0, 100.0, 0.0)
    assert lat > -35.363, "Forward offset North should increase lat"
    assert abs(lat - (-35.363)) > 0.0008
test("forward offset heading North increases latitude", test_offset_north_forward)


def test_offset_east_right():
    # Heading North, move 100 m right → East → lon increases
    lat, lon = _apply_ned_offset(-35.363, 149.165, 0.0, 0.0, 100.0)
    assert lon > 149.165, "Right offset heading North should increase lon (East)"
test("right offset heading North increases longitude", test_offset_east_right)


def test_offset_east_forward():
    # Heading East (90°), move 50 m forward → lon increases
    lat, lon = _apply_ned_offset(-35.363, 149.165, 90.0, 50.0, 0.0)
    assert lon > 149.165, "Forward offset heading East should increase lon"
test("forward offset heading East increases longitude", test_offset_east_forward)


def test_offset_east_right_is_south():
    # Heading East (90°), move 50 m right → South → lat decreases
    lat, lon = _apply_ned_offset(-35.363, 149.165, 90.0, 0.0, 50.0)
    assert lat < -35.363, "Right offset heading East should be South (decrease lat)"
test("right offset heading East decreases latitude (South)", test_offset_east_right_is_south)


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------

print()
print("── Results ──────────────────────────────────────────────────────────")
print(f"  Passed: {passed}  Failed: {failed}")
sys.exit(0 if failed == 0 else 1)
