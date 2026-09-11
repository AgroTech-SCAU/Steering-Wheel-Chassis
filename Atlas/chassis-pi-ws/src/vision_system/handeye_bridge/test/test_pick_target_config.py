import pytest
from types import SimpleNamespace

from handeye_bridge.pick_target_config import resolve_pick_target_parameters


def test_pick_target_explicit_z_and_orientation_override_defaults():
    msg = SimpleNamespace(
        use_target_z=True,
        target_z_m=0.123,
        use_orientation=True,
        pitch_rad=-1.1,
        yaw_rad=0.7,
    )

    result = resolve_pick_target_parameters(msg, 0.05, -3.0, 3.0)

    assert result == (0.123, -1.1, 0.7, True)


def test_pick_target_legacy_message_keeps_defaults():
    msg = SimpleNamespace()

    result = resolve_pick_target_parameters(msg, 0.05, -3.0, 3.0)

    assert result == (0.05, -3.0, 3.0, False)


def test_pick_target_explicit_approach_is_separate_from_projection_plane():
    from handeye_bridge.pick_target_config import pick_command_z

    msg = SimpleNamespace(use_approach=True, approach_m=0.05)
    assert pick_command_z(msg, plane_z=0.12, default_offset_m=0.003) == pytest.approx(0.17)


def test_pick_target_legacy_command_z_keeps_default_offset():
    from handeye_bridge.pick_target_config import pick_command_z

    msg = SimpleNamespace()
    assert pick_command_z(msg, plane_z=0.12, default_offset_m=0.003) == pytest.approx(0.123)
