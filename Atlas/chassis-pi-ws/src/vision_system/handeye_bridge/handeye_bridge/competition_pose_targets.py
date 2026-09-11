from __future__ import annotations

from atlas_competition_config.config import CompetitionConfigError, resolve_arm_pose


def pickup_vision_pose_targets(arm_motion: dict) -> list[tuple[str, tuple[float, float, float]]]:
    targets: list[tuple[str, tuple[float, float, float]]] = []
    for arena in ("A", "B"):
        try:
            pose = resolve_arm_pose(arm_motion, "pickup_observe", arena=arena)
        except CompetitionConfigError:
            continue
        targets.append(
            (
                f"pickup_observe_{arena.lower()}",
                (float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"])),
            )
        )
    return targets
