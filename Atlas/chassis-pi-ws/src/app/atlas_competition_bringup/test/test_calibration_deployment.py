from pathlib import Path
import importlib.util

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "calibration_deployment.py"
spec = importlib.util.spec_from_file_location("calibration_deployment", MODULE_PATH)
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)


def test_deploy_backs_up_active_config_and_preserves_symlink(tmp_path):
    active = tmp_path / "competition.yaml"
    active.write_text("competition:\n  backend_name: old\n", encoding="utf-8")
    installed = tmp_path / "installed.yaml"
    installed.symlink_to(active)
    exported = tmp_path / "exported.yaml"
    exported.write_text("competition:\n  backend_name: new\n", encoding="utf-8")

    backup = deployment.deploy_calibration(exported, installed)

    assert installed.is_symlink()
    assert active.read_text(encoding="utf-8") == exported.read_text(encoding="utf-8")
    assert backup.read_text(encoding="utf-8") == "competition:\n  backend_name: old\n"


def test_deploy_rejects_invalid_export_without_touching_active_config(tmp_path):
    active = tmp_path / "competition.yaml"
    active.write_text("competition: {}\n", encoding="utf-8")
    exported = tmp_path / "invalid.yaml"
    exported.write_text("other: true\n", encoding="utf-8")

    with pytest.raises(ValueError, match="缺少 competition"):
        deployment.deploy_calibration(exported, active)

    assert active.read_text(encoding="utf-8") == "competition: {}\n"
    assert not list(tmp_path.glob("*.backup-*"))
