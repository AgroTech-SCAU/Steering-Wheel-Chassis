from __future__ import annotations

from atlas_competition_config.config import CompetitionConfigError, resolve_arm_pose


def pickup_vision_pose_targets(arm_motion: dict) -> list[tuple[str, tuple[float, float, float]]]:
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
                )
            )
    return targets


def park_vision_pose_targets(arm_motion: dict) -> list[tuple[str, tuple[float, float, float]]]:
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
                )
            )
    return targets
