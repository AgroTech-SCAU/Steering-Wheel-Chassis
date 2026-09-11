from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parents[1]


def test_competition_stack_starts_direct_backend_by_default():
    launch = (ROOT / "launch/competition_stack.launch.py").read_text()
    assert "atlas_nav_direct_backend" in launch
    assert "direct_nav_backend" in launch
    assert "direct_odom_competition" in launch
    assert "atlas_nav_full_backend" not in launch


def test_competition_yaml_selects_direct_backend_and_documents_origin_alignment():
    config = (ROOT / "config/competition.yaml").read_text()
    assert "backend_name: direct_odom_competition" in config
    assert "startup_localization:" in config
    assert "中转区最优位姿" in config


def test_bringup_declares_direct_backend_dependency():
    package = (ROOT / "package.xml").read_text()
    assert "<exec_depend>atlas_nav_direct_backend</exec_depend>" in package


def test_nav_readme_explains_lidar_once_then_odom_direct():
    readme = (SRC / "nav_system/README.md").read_text()
    assert "atlas_nav_direct_backend" in readme
    assert "一次" in readme
    assert "odom" in readme.lower()
