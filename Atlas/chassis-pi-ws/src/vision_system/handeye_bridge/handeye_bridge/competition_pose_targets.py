from __future__ import annotations

from atlas_competition_config.config import (
    CompetitionConfigError,
    resolve_arm_pose,
    resolve_pickup_layer_z,
)


def pickup_vision_pose_targets(arm_motion: dict, *, include_axis=False) -> list[tuple[str, tuple[float, float, float]]]:
    targets: list[tuple[str, tuple[float, float, float]]] = []
    for arena in ("A", "B"):
        pickup = arm_motion.get("arenas", {}).get(arena, {}).get("pickup", {})
        slots = range(4) if "observations" in pickup else (None,)
        for slot in slots:
            try:
                pose = resolve_arm_pose(arm_motion, "pickup_observe", arena=arena, slot=slot)
            except CompetitionConfigError:
                continue
            suffix = f"_slot{slot}" if slot is not None else ""
            targets.append(
                (
                    f"pickup_observe_{arena.lower()}{suffix}",
                    (float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"])),
                    *((calibrated_tool_axis(pose),) if include_axis else ()),
                )
            )
    return targets


def pickup_plane_calibrations(
    arm_motion: dict,
) -> dict[str, dict[str, tuple[float, float]]]:
    """Return per-observation contact Z and derived observation distances."""
    result = {}
    for arena in ("A", "B"):
        pickup = arm_motion.get("arenas", {}).get(arena, {}).get("pickup", {})
        slots = range(4) if "observations" in pickup else (None,)
        for slot in slots:
            try:
                pose = resolve_arm_pose(arm_motion, "pickup_observe", arena=arena, slot=slot)
                layer_z = tuple(
                    resolve_pickup_layer_z(arm_motion, arena, layer, slot=slot)
                    for layer in (1, 2)
                )
            except CompetitionConfigError:
                continue
            suffix = "" if slot is None else f"_slot{slot}"
            name = f"pickup_observe_{arena.lower()}{suffix}"
            observe_z = float(pose["z_m"])
            result[name] = {
                "layer_z_m": layer_z,
                "camera_to_plane_distance_m": tuple(
                    round(observe_z - z, 6) for z in layer_z
                ),
            }
    return result


def park_vision_pose_targets(arm_motion: dict, *, include_axis=False) -> list[tuple[str, tuple[float, float, float]]]:
    targets: list[tuple[str, tuple[float, float, float]]] = []
    for arena in ("A", "B"):
        for area in ("park_1", "park_2"):
            try:
                pose = resolve_arm_pose(arm_motion, "park_prepare", arena=arena, area=area)
            except CompetitionConfigError:
                continue
            targets.append(
                (
                    f"{area}_prepare_{arena.lower()}",
                    (float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"])),
                    *((calibrated_tool_axis(pose),) if include_axis else ()),
                )
            )
    return targets


def calibrated_tool_axis(pose):
    """Reuse the calibrated FK shipped from calib, ignoring legacy Euler fields"""
    import importlib.util
    from pathlib import Path
    source = Path(__file__).resolve().parents[2] / "calib" / "fk_utils.py"
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory("handeye_bridge")) / "calib" / "fk_utils.py"
    global _fk_module
    if _fk_module is None:
        spec = importlib.util.spec_from_file_location("handeye_calibrated_fk", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _fk_module = module
    return _fk_module.fk_gripper_in_base(pose["joints_rad"])[:3, 2]


_fk_module = None
