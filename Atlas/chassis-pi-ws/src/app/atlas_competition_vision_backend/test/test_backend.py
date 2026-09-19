from types import SimpleNamespace

from atlas_competition_vision_backend.backend import (
    BackendConfig,
    CompetitionVisionBackend,
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
        _config(enabled=False), "A",
    )

    assert not result.success


def test_sorting_rule_maps_image_sides_by_arena():
    gear_on_left = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("luosi", 125.0, 25.0, 0.9),
        ],
        _config(), "A",
    )
    t_bolt_on_left = resolve_sorting_rule(
        [
            Detection("luosi", 25.0, 25.0, 0.9),
            Detection("chilun", 125.0, 25.0, 0.9),
        ],
        _config(), "B",
    )

    assert gear_on_left.success
    assert gear_on_left.park_1_cargo == "t_bolt"
    assert gear_on_left.park_2_cargo == "gear"
    assert t_bolt_on_left.success
    assert t_bolt_on_left.park_1_cargo == "t_bolt"
    assert t_bolt_on_left.park_2_cargo == "gear"


def test_a_left_luosi_right_chilun_routes_by_observed_class():
    result = classify_with_scan_sequence(
        lambda _scan: [
            Detection("luosi", 25.0, 25.0),
            Detection("chilun", 125.0, 25.0),
        ],
        _config(),
    )
    assert result.success and result.arena == "A"
    assert result.park_1_cargo == "gear"
    assert result.park_2_cargo == "t_bolt"


def test_sorting_rule_rejects_more_than_two_parts():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 15.0, 40.0, 0.2),
            Detection("chilun", 140.0, 40.0, 0.95),
            Detection("luosi", 60.0, 40.0, 0.90),
        ],
        _config(), "B",
    )

    assert not result.success
    assert "exactly two" in result.message


def test_sorting_rule_requires_both_cargo_classes():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 25.0, 25.0, 0.9),
            Detection("chilun", 125.0, 25.0, 0.8),
        ],
        _config(), "A",
    )

    assert not result.success
    assert "both" in result.message


def test_sorting_rule_rejects_equal_horizontal_centers_as_ambiguous():
    result = resolve_sorting_rule(
        [
            Detection("chilun", 80.0, 25.0, 0.9),
            Detection("luosi", 80.0, 75.0, 0.9),
        ],
        _config(), "A",
    )

    assert not result.success
    assert "ambiguous" in result.message


def test_scan_a_success_stops_before_scan_b():
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
    assert calls == ["sorting_scan_a"]
    assert result.park_1_cargo == "t_bolt"
    assert result.park_2_cargo == "gear"


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
    assert result.park_1_cargo == "gear"
    assert result.park_2_cargo == "t_bolt"


def test_scan_a_device_failure_does_not_select_b():
    calls = []

    def scan(name):
        calls.append(name)
        return None if name == "sorting_scan_a" else [
            Detection("chilun", 25.0, 25.0),
            Detection("luosi", 125.0, 25.0),
        ]

    result = classify_with_scan_sequence(scan, _config())
    assert not result.success
    assert calls == ["sorting_scan_a"]


def test_only_current_capture_final_centers_are_used():
    backend = CompetitionVisionBackend.__new__(CompetitionVisionBackend)
    backend._latest_centers = []
    backend._final_centers = None
    backend._capture_start_ns = 2_000_000_000
    detection = SimpleNamespace(
        cls_name="chilun", u=20.0, v=30.0, conf=0.9, corner_index=0,
    )

    def publish(stamp_sec, is_final):
        backend._on_detection_centers(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=0)),
            is_final_best=is_final, detections=[detection],
        ))

    publish(1, True)
    assert backend._final_centers is None
    publish(2, False)
    assert backend._final_centers is None
    publish(2, True)
    assert backend._final_centers == [Detection("chilun", 20.0, 30.0, 0.9, 0)]


def test_target_capture_reports_detector_start_failure():
    backend = CompetitionVisionBackend.__new__(CompetitionVisionBackend)
    backend._wait_observation_pose = lambda: True
    backend._final_centers = None
    backend._capture_start_ns = 0
    backend.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=42))
    backend._set_vision_detect = lambda _start: (
        SimpleNamespace(success=False, message="camera unavailable"), "",
    )
    backend.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    centers, error = backend._capture_target_centers()

    assert centers is None
    assert error == "camera unavailable"
    assert backend._capture_start_ns == 0


def test_observation_pose_must_remain_ready_after_settling(monkeypatch):
    backend = CompetitionVisionBackend.__new__(CompetitionVisionBackend)
    backend._vision_pose_ready = True
    backend._wait_vision_pose_ready = lambda: True
    backend.get_parameter = lambda _name: SimpleNamespace(value=0.3)
    monkeypatch.setattr(
        "atlas_competition_vision_backend.backend.time.sleep",
        lambda _seconds: setattr(backend, "_vision_pose_ready", False),
    )

    assert not backend._wait_observation_pose()


def test_target_service_preserves_capture_failure():
    backend = CompetitionVisionBackend.__new__(CompetitionVisionBackend)
    backend._capture_target_centers = lambda: (None, "vision_detect service timeout")
    response = SimpleNamespace(
        success=True, layer_ok=True, complete=True, message="", target_count=1,
    )

    backend._on_detect_target(SimpleNamespace(), response)

    assert not response.success
    assert not response.layer_ok
    assert not response.complete
    assert response.message == "vision_detect service timeout"
    assert response.target_count == 0


def test_scan_a_valid_does_not_consult_scan_b():
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


def test_pickup_slot_view_accepts_one_visible_part_at_slot_three():
    request = SimpleNamespace(
        waypoint_id="pickup", slot=3, expected_layer=2,
        max_targets=1, target_class="",
    )
    result = detect_camera_target_from_centers(
        request, [Detection("luosi", 80.0, 90.0, 0.9, corner_index=0)],
        _config().class_aliases,
    )
    assert result.success and result.complete
    assert result.cargo_class == "t_bolt"


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
