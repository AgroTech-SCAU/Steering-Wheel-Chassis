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
            "sorting_rule": {"enabled": enabled},
        }
    )


def test_sorting_rule_disabled_fails_safe():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ],
        _config(enabled=False),
    )

    assert not result.success


def test_sorting_rule_maps_image_left_to_park_1_and_right_to_park_2():
    gear_on_left = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ],
        _config(),
    )
    t_bolt_on_left = resolve_sorting_rule(
        [
            Detection("luosi", 25.0, 25.0, 0.9),
            Detection("chilun", 125.0, 25.0, 0.9),
        ],
        _config(),
    )

    assert gear_on_left.success
    assert gear_on_left.park_1_cargo == "gear"
    assert gear_on_left.park_2_cargo == "t_bolt"
    assert t_bolt_on_left.success
    assert t_bolt_on_left.park_1_cargo == "t_bolt"
    assert t_bolt_on_left.park_2_cargo == "gear"


def test_sorting_rule_uses_best_detection_per_cargo_class_before_left_right_ordering():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 15.0, 40.0, 0.2),
            Detection("chilun", 140.0, 40.0, 0.95),
            Detection("luosi", 60.0, 40.0, 0.90),
        ],
        _config(),
    )

    assert result.success
    assert result.park_1_cargo == "t_bolt"
    assert result.park_2_cargo == "gear"


def test_sorting_rule_requires_both_cargo_classes():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("chilun", 125.0, 25.0, 0.8),
        ],
        _config(),
    )

    assert not result.success
    assert "both" in result.message


def test_sorting_rule_rejects_equal_horizontal_centers_as_ambiguous():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 80.0, 25.0, 0.9),
            Detection("luosi", 80.0, 75.0, 0.9),
        ],
        _config(),
    )

    assert not result.success
    assert "ambiguous" in result.message


def test_scan_a_success_still_checks_scan_b_and_returns_a_when_unique():
    calls = []

    def scan(name):
        calls.append(name)
        if name == "sorting_scan_b":
            return []
        return [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ]

    result = classify_with_scan_sequence(scan, _config())

    assert result.success
    assert result.arena == "A"
    assert calls == ["sorting_scan_a", "sorting_scan_b"]


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


def test_scan_a_and_scan_b_both_valid_is_rejected_as_ambiguous():
    calls = []

    def scan(name):
        calls.append(name)
        return [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ]

    result = classify_with_scan_sequence(scan, _config())

    assert not result.success
    assert result.arena == ""
    assert "ambiguous" in result.message
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


def test_load_yaml_config_accepts_left_right_sorting_without_roi(tmp_path):
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
""",
        encoding="utf-8",
    )

    config = load_yaml_config(str(config_path))

    assert config.sorting_enabled is True
    assert not hasattr(config, "park_1_roi")
    assert not hasattr(config, "park_2_roi")
