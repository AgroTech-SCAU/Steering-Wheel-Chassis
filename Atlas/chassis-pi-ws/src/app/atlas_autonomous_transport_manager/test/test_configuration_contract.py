"""全自主运输配置的静态一致性检查"""

from pathlib import Path

import math
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[1]
ACTIONS_PATH = (
    WORKSPACE_SRC
    / 'vision_system'
    / 'atlas_vision_pollination_backend'
    / 'config'
    / 'transport_actions.yaml'
)
MANAGER_PATH = PACKAGE_ROOT / 'config' / 'autonomous_transport.yaml'
FULL_NAV_PATH = PACKAGE_ROOT / 'config' / 'autonomous_full_nav.yaml'

SORTING_RULE_PATH = (
    WORKSPACE_SRC
    / 'vision_system'
    / 'racom_vision'
    / 'atlas_racom_vision_backend'
    / 'config'
    / 'sorting_rule.yaml'
)


def load_yaml(path: Path) -> dict:
    """读取 YAML 并确保根节点为字典"""
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert isinstance(data, dict), f'{path} 根节点必须是字典'
    return data


def test_cargo_plan_and_expected_counts_are_consistent() -> None:
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    cargo = root['cargo']
    expected = cargo['expected_counts']

    assert cargo['target_total'] == sum(expected.values())
    assert len(cargo['plan']) >= cargo['target_total']
    assert set(cargo['plan']) <= {'gear', 't_bolt'}
    assert expected['gear'] == 4
    assert expected['t_bolt'] == 4


def test_all_waypoints_are_finite_and_have_positive_timeouts() -> None:
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    waypoints = root['waypoints']

    assert set(waypoints) == {'sorting_area', 'dispatch_area', 'park_1', 'park_2'}
    for name, waypoint in waypoints.items():
        values = [
            float(waypoint['x_m']),
            float(waypoint['y_m']),
            float(waypoint['yaw_rad']),
            float(waypoint['timeout_s']),
        ]
        assert all(math.isfinite(value) for value in values), name
        assert waypoint['timeout_s'] > 0.0, name


def test_manager_task_references_exist_in_action_config() -> None:
    manager = load_yaml(MANAGER_PATH)['autonomous_transport']['manipulation']
    actions = load_yaml(ACTIONS_PATH)
    prepare_actions = actions['prepare_actions']
    arrival_tasks = actions['arrival_tasks']

    assert manager['safe_prepare_action'] in prepare_actions
    assert manager['sorting_prepare_action'] in prepare_actions
    assert 'prepare_only' in arrival_tasks

    for action_name in manager['pick_prepare_actions'].values():
        assert action_name in prepare_actions
    for task_name in manager['pick_tasks'].values():
        assert task_name in arrival_tasks

    for action_name in manager['place_prepare_actions'].values():
        assert action_name in prepare_actions
    for park_tasks in manager['place_tasks'].values():
        assert len(park_tasks) >= 4
        for task_name in park_tasks:
            assert task_name in arrival_tasks


def test_action_references_are_resolvable() -> None:
    actions = load_yaml(ACTIONS_PATH)
    prepare_actions = actions['prepare_actions']
    arrival_tasks = actions['arrival_tasks']

    for task_name, task in arrival_tasks.items():
        sequences = [task.get('sequence', []), task.get('per_target_sequence', [])]
        sequences.append(task.get('after_all_targets_sequence', []))
        for sequence in sequences:
            for step in sequence:
                if step.get('type') != 'joints_action':
                    continue
                action_ref = step.get('action_ref', 'prepare_action')
                if action_ref == 'prepare_action':
                    continue
                assert action_ref in prepare_actions, (task_name, action_ref)


def test_calibration_gate_is_closed_in_distributed_config() -> None:
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    assert root['calibration_confirmed'] is False



def test_classification_confidence_gate_requires_complete_mapping() -> None:
    """默认阈值允许双标识完整识别，并拒绝单标识互补推断"""
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    minimum = float(root['classification']['minimum_confidence'])
    sorting = load_yaml(SORTING_RULE_PATH)['atlas_sorting_rule_service']['ros__parameters']

    assert 0.45 < minimum <= 0.75
    assert sorting['allow_complement_inference'] is False


def test_full_navigation_backend_uses_absolute_map_coordinates() -> None:
    """管理器下发 map 坐标时，完整导航后端必须按绝对地图坐标解释"""
    root = load_yaml(FULL_NAV_PATH)
    parameters = root['atlas_nav_full_backend']['ros__parameters']

    assert parameters['coordinate_mode'] == 'absolute_map'
    assert parameters['map_frame'] == 'map'



def test_asrpro_voice_gate_matches_competition_flow() -> None:
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    voice = root['voice']

    assert voice['start_required'] is True
    assert float(voice['start_timeout_s']) == 0.0
    assert voice['fallback_enabled'] is False
    assert 'atlas_start' in voice['accepted_intents']
    assert voice['phrase_ids']['transition_complete'] == 'transition_complete'
    assert voice['phrase_ids']['autonomous_start'] == 'autonomous_start'


def test_recoverable_stage_policy_is_enabled() -> None:
    root = load_yaml(MANAGER_PATH)['autonomous_transport']
    recovery = root['recovery']

    assert int(recovery['navigation_retry_count']) >= 0
    assert int(recovery['classification_retry_count']) >= 0
    assert int(recovery['pick_retry_count']) >= 0
    assert recovery['continue_on_pick_failure'] is True


def test_latest_vision_model_path_is_centralized() -> None:
    root = load_yaml(MANAGER_PATH)
    system = root['system']

    assert 'vision_model_path' in system['paths']
    assert 'vision_labels_path' in system['paths']
    assert system['vision']['camera'] == '/dev/atlas_camera'
