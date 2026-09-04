from atlas_competition_manipulation_backend.backend import (
    load_placement_config,
)


def test_load_placement_config_accepts_top_level_competition_section(tmp_path):
    """Catches the backend ignoring competition.manipulation.placement."""
    config_path = tmp_path / "competition.yaml"
    config_path.write_text(
        """
competition:
  manipulation:
    placement:
      enabled: true
      approach_m: 0.08
      layer_step_m: 0.045
      park_1: {x_m: 0.31, y_m: 0.11, first_layer_z_m: 0.06}
      park_2: {x_m: 0.42, y_m: -0.12, first_layer_z_m: 0.07}
      slot_offsets_xy_m: [0.0, 0.0, 0.05, 0.0, 0.05, 0.05, 0.0, 0.05]
""",
        encoding="utf-8",
    )

    placement = load_placement_config(str(config_path))

    assert placement["enabled"] is True
    assert placement["approach_m"] == 0.08
    assert placement["layer_step_m"] == 0.045
    assert placement["park_1"]["x_m"] == 0.31
    assert placement["park_2"]["y_m"] == -0.12
