"""Deploy an exported calibration to the config path used by the launch file."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import tempfile

import yaml


def deploy_calibration(export_path: str | Path, config_path: str | Path) -> Path:
    """Back up the active config, then atomically replace its resolved file."""
    exported = Path(export_path).expanduser().resolve(strict=True)
    active = Path(config_path).expanduser().resolve(strict=True)
    if exported == active:
        raise ValueError("导出文件与当前运行配置是同一个文件")

    content = exported.read_text(encoding="utf-8")
    parsed = yaml.safe_load(content)
    if not isinstance(parsed, dict) or not isinstance(parsed.get("competition"), dict):
        raise ValueError("导出文件缺少 competition 配置，拒绝部署")

    backup_fd, backup_name = tempfile.mkstemp(
        dir=active.parent, prefix=f"{active.name}.backup-"
    )
    os.close(backup_fd)
    backup = Path(backup_name)
    try:
        shutil.copy2(active, backup)
    except OSError:
        backup.unlink(missing_ok=True)
        raise

    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=active.parent,
            prefix=f".{active.name}.deploy-", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        shutil.copymode(active, temporary)
        os.replace(temporary, active)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return backup
