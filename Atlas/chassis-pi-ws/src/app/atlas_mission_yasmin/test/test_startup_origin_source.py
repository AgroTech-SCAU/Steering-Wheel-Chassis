from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_autonomous_machine_inspects_arena_before_origin_map_matching():
    machine = (ROOT / "src/machine.cpp").read_text()
    assert '"NAV_ORIGIN", std::make_shared<NavOriginState>' in machine
    assert 'action_transitions("INSPECT_SORT_ZONE")' in machine
    assert 'action_transitions("NAV_ORIGIN")' in machine
    nav_pos = machine.index('"NAV_ORIGIN"')
    arm_pos = machine.index('"ARM_ZERO"')
    inspect_pos = machine.index('"INSPECT_SORT_ZONE"')
    assert arm_pos < inspect_pos < nav_pos


def test_runtime_requires_arena_before_origin_navigation():
    runtime = (ROOT / "src/runtime.cpp").read_text()
    assert 'if (model_.arena() != "A" && model_.arena() != "B")' in runtime
    assert 'waypoint_id != "origin"' not in runtime


def test_runtime_plan_contains_origin_waypoint():
    header = (ROOT / "include/atlas_mission_yasmin/runtime.hpp").read_text()
    runtime = (ROOT / "src/runtime.cpp").read_text()
    route = (ROOT / "config/mission_route.yaml").read_text()
    assert "Waypoint origin;" in header
    assert 'waypoints["origin"]' in runtime
    assert 'if (id == "origin")' in runtime
    assert "origin:" in route
    assert "direct_odom_competition" in route


def test_nav_origin_state_uses_existing_navigation_request_path():
    states_h = (ROOT / "include/atlas_mission_yasmin/states.hpp").read_text()
    states_cpp = (ROOT / "src/states.cpp").read_text()
    assert "class NavOriginState final" in states_h
    assert 'runtime_->navigate("origin")' in states_cpp
