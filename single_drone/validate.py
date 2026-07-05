#!/usr/bin/env python3
"""
Quick unit tests for the pipeline stages 
Run: python3 validate.py
Used in initial testing of the mission validator and LLM planner. Not part of the main submission.

"""

import json
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from src.mission_validator import validate_mission, ValidationError

PASS = "\033[32m✓\033[0m"
FAIL = "\033[31m✗\033[0m"
passed = failed = 0


def test(name, fn):
    global passed, failed
    try:
        fn()
        print(f"  {PASS} {name}")
        passed += 1
    except Exception as e:
        print(f"  {FAIL} {name}: {e}")
        failed += 1


def load(fname):
    with open(f"missions/examples/{fname}") as f:
        return json.load(f)


def mutate(mission, **kwargs):
    import copy
    m = copy.deepcopy(mission)
    for k, v in kwargs.items():
        keys = k.split(".")
        obj = m
        for key in keys[:-1]:
            obj = obj[key]
        obj[keys[-1]] = v
    return m


print("\n── Validator Tests ──────────────────────────────────────────")

test("patrol_loop.json is valid", lambda: validate_mission(load("patrol_loop.json")))
test("inspection_grid.json is valid", lambda: validate_mission(load("inspection_grid.json")))

def test_alt_too_high():
    m = mutate(load("patrol_loop.json"), **{"parameters.altitude_m": 200})
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("altitude 200m → rejected", test_alt_too_high)

def test_alt_too_low():
    m = mutate(load("patrol_loop.json"), **{"parameters.altitude_m": 1})
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("altitude 1m → rejected", test_alt_too_low)

def test_speed_too_high():
    m = mutate(load("patrol_loop.json"), **{"parameters.groundspeed_ms": 25})
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("speed 25m/s → rejected", test_speed_too_high)

def test_geofence():
    import copy
    m = copy.deepcopy(load("patrol_loop.json"))
    m["waypoints"][0]["lat"] = -34.0   # North of geofence
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("waypoint outside geofence → rejected", test_geofence)

def test_missing_field():
    import copy
    m = copy.deepcopy(load("patrol_loop.json"))
    del m["parameters"]
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("missing 'parameters' field → rejected", test_missing_field)

def test_empty_waypoints():
    import copy
    m = copy.deepcopy(load("patrol_loop.json"))
    m["waypoints"] = []
    try:
        validate_mission(m)
        raise AssertionError("Should have been rejected")
    except ValidationError:
        pass
test("empty waypoints → rejected", test_empty_waypoints)

print()
print(f"── LLM Planner (mock) ───────────────────────────────────────")

def test_planner_mock():
    from src.llm_planner import plan_mission
    import unittest.mock as mock

    mock_json = json.dumps({
        "mission_id": "00000000-0000-0000-0000-000000000001",
        "created_at": "2025-01-15T10:00:00Z",
        "natural_language_input": "test",
        "vehicle_id": "copter_1",
        "home_location": {"lat": -35.363261, "lon": 149.165230, "alt_m": 0},
        "parameters": {"altitude_m": 15, "groundspeed_ms": 5, "loops": 1,
                       "return_to_home": True, "loiter_time_s": 0},
        "waypoints": [
            {"id": 1, "lat": -35.362, "lon": 149.165, "alt_m": 15, "action": "none", "hold_s": 0}
        ]
    })

    # Mock the OpenAI-compatible client (aicredits.in / DeepSeek)
    with mock.patch("src.llm_planner._get_client") as mock_get_client:
        mock_client = mock.MagicMock()
        mock_get_client.return_value = mock_client
        mock_response = mock.MagicMock()
        mock_response.choices[0].message.content = mock_json
        mock_client.chat.completions.create.return_value = mock_response
        mission = plan_mission("test prompt")

    assert mission["parameters"]["altitude_m"] == 15
    assert len(mission["waypoints"]) == 1
    # Verify UUID is freshly stamped
    assert mission["mission_id"] != "00000000-0000-0000-0000-000000000001"

test("LLM planner parses JSON and stamps fresh UUID", test_planner_mock)

def test_planner_strips_markdown():
    from src.llm_planner import plan_mission
    import unittest.mock as mock

    mock_md = "```json\n" + json.dumps({
        "mission_id": "00000000-0000-0000-0000-000000000002",
        "created_at": "2025-01-15T10:00:00Z",
        "natural_language_input": "test",
        "vehicle_id": "copter_1",
        "home_location": {"lat": -35.363261, "lon": 149.165230, "alt_m": 0},
        "parameters": {"altitude_m": 10, "groundspeed_ms": 3, "loops": 1,
                       "return_to_home": False, "loiter_time_s": 0},
        "waypoints": [
            {"id": 1, "lat": -35.362, "lon": 149.165, "alt_m": 10, "action": "none", "hold_s": 0}
        ]
    }) + "\n```"

    with mock.patch("src.llm_planner._get_client") as mock_get_client:
        mock_client = mock.MagicMock()
        mock_get_client.return_value = mock_client
        mock_response = mock.MagicMock()
        mock_response.choices[0].message.content = mock_md
        mock_client.chat.completions.create.return_value = mock_response
        mission = plan_mission("test")

    assert mission["parameters"]["altitude_m"] == 10

test("LLM planner strips markdown code fences", test_planner_strips_markdown)

print()
print(f"── Results ──────────────────────────────────────────────────")
print(f"  Passed: {passed}  Failed: {failed}")
sys.exit(0 if failed == 0 else 1)
