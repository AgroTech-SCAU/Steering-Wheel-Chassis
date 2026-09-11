from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_autonomous_machine_keeps_arm_zero_first_then_corrects_origin_before_scan():
    machine = (ROOT / "src/machine.cpp").read_text()
    assert '"NAV_ORIGIN", std::make_shared<NavOriginState>' in machine
    assert 'action_transitions("NAV_ORIGIN")' in machine
    assert 'action_transitions("INSPECT_SORT_ZONE")' in machine
    nav_pos = machine.index('"NAV_ORIGIN"')
    arm_pos = machine.index('"ARM_ZERO"')
    inspect_pos = machine.index('"INSPECT_SORT_ZONE"')
    assert arm_pos < nav_pos < inspect_pos


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
