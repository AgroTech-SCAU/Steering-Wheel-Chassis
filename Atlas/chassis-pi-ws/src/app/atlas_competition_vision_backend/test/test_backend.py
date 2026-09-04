from types import SimpleNamespace

from atlas_competition_vision_backend.backend import (
    BackendConfig,
    Detection,
    classify_with_scan_sequence,
    detect_camera_target_from_centers,
    load_yaml_config,
    resolve_sorting_rule,
)


def _config(enabled=True):
    return BackendConfig.from_dict(
        {
            "class_aliases": {"chilun": "gear", "luosi": "t_bolt"},
            "sorting_rule": {
                "enabled": enabled,
                "park_1_roi": [0, 0, 100, 100],
                "park_2_roi": [100, 0, 200, 100],
            },
        }
    )


def test_sorting_rule_roi_disabled_fails_safe():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ],
        _config(enabled=False),
    )

    assert not result.success


def test_sorting_rule_decodes_both_park_mappings_from_roi():
    gear_to_park_1 = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ],
        _config(),
    )
    gear_to_park_2 = resolve_sorting_rule(
        [
            Detection("luosi", 25.0, 25.0, 0.9),
            Detection("chilun", 125.0, 25.0, 0.9),
        ],
        _config(),
    )

    assert gear_to_park_1.success
    assert gear_to_park_1.park_1_cargo == "gear"
    assert gear_to_park_1.park_2_cargo == "t_bolt"
    assert gear_to_park_2.success
    assert gear_to_park_2.park_1_cargo == "t_bolt"
    assert gear_to_park_2.park_2_cargo == "gear"


def test_scan_a_success_returns_a_without_scan_b():
    calls = []

    def scan(name):
        calls.append(name)
        return [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ]

    result = classify_with_scan_sequence(scan, _config())

    assert result.success
    assert result.arena == "A"
    assert calls == ["sorting_scan_a"]


def test_scan_a_failure_then_scan_b_success_returns_b():
    calls = []

    def scan(name):
        calls.append(name)
        if name == "sorting_scan_a":
            return []
        return [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ]

    result = classify_with_scan_sequence(scan, _config())

    assert result.success
    assert result.arena == "B"
    assert calls == ["sorting_scan_a", "sorting_scan_b"]


def test_scan_a_and_scan_b_fail_when_no_unique_rule_is_visible():
    result = classify_with_scan_sequence(lambda _name: [], _config())

    assert not result.success
    assert result.arena == ""


def test_detect_camera_target_selects_each_slot_by_corner_index():
    centers = [
        Detection("chilun", 10.0, 10.0, 0.9, corner_index=0),
        Detection("luosi", 20.0, 10.0, 0.8, corner_index=1),
        Detection("chilun", 20.0, 20.0, 0.7, corner_index=2),
        Detection("luosi", 10.0, 20.0, 0.6, corner_index=3),
    ]

    assert [
        detect_camera_target_from_centers(
            SimpleNamespace(slot=slot, expected_layer=1, max_targets=1, target_class=""),
            centers,
            _config().class_aliases,
        ).cargo_class
        for slot in range(4)
    ] == ["gear", "t_bolt", "gear", "t_bolt"]


def test_load_yaml_config_accepts_top_level_competition_vision_section(tmp_path):
    """Catches the backend ignoring competition.vision in the single top-level YAML."""
    config_path = tmp_path / "competition.yaml"
    config_path.write_text(
        """
competition:
  vision:
    class_aliases:
      chilun: gear
      luosi: t_bolt
    sorting_rule:
      enabled: true
      park_1_roi: [10, 20, 110, 120]
      park_2_roi: [130, 20, 230, 120]
""",
        encoding="utf-8",
    )

    config = load_yaml_config(str(config_path))

    assert config.sorting_enabled is True
    assert config.park_1_roi == (10.0, 20.0, 110.0, 120.0)
    assert config.park_2_roi == (130.0, 20.0, 230.0, 120.0)
