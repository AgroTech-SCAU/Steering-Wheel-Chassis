from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parents[1]


def test_competition_stack_starts_direct_backend_by_default():
    launch = (ROOT / "launch/competition_stack.launch.py").read_text()
    assert "atlas_nav_direct_backend" in launch
    assert "direct_nav_backend" in launch
    assert "direct_odom_competition" in launch
    assert "atlas_nav_full_backend" not in launch


def test_auto_zone_test_keeps_mission_origin_localization_enabled():
    launch = (ROOT / "launch/auto_zone_test.launch.py").read_text()
    assert '"competition_stack.launch.py"' in launch
    assert '"enable_navigation": "true"' in launch
    assert '"enable_mission": "true"' in launch
    assert '"navigation_backend_name": "direct_odom_competition"' in launch


def test_competition_yaml_selects_direct_backend_and_documents_origin_alignment():
    config = (ROOT / "config/competition.yaml").read_text()
    assert "backend_name: direct_odom_competition" in config
    assert "startup_localization:" in config
    assert "中转区最优位姿" in config


def test_bringup_declares_direct_backend_dependency():
    package = (ROOT / "package.xml").read_text()
    assert "<exec_depend>atlas_nav_direct_backend</exec_depend>" in package


def test_nav_readme_explains_continuous_lidar_correction_and_odom_direct():
    readme = (SRC / "nav_system/README.md").read_text()
    assert "atlas_nav_direct_backend" in readme
    assert "持续校正" in readme
    assert "odom" in readme.lower()


def test_autonomous_speed_profiles_match_mcu_remote_mid_chassis_and_fast_arm():
    competition = yaml.safe_load((ROOT / "config/competition.yaml").read_text())["competition"]
    direct = competition["navigation"]["direct_control"]
    assert direct["max_linear_speed_m_s"] == 1.0
    assert direct["max_angular_speed_rad_s"] == 4.0
    assert competition["handeye_bridge"]["default_speed_rad_s"] == 50.24

    def pose_speeds(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == "speed_rad_s":
                    yield float(child)
                else:
                    yield from pose_speeds(child)
        elif isinstance(value, list):
            for child in value:
                yield from pose_speeds(child)

    speeds = list(pose_speeds(competition["arm_motion"]))
    assert speeds
    assert set(speeds) == {50.24}

    mcu_control = (
        SRC.parents[1] / "chassis_control_code/src/app/app_control.c"
    ).read_text()
    assert "#define REMOTE_MID_MAX_VX_MPS 1.0f" in mcu_control
    assert "#define REMOTE_MID_MAX_WZ_RAD_S 4.0f" in mcu_control
    assert "limit.servo_speed_rad_s = 50.24f;" in mcu_control
