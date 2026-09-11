from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
from typing import Iterable, Mapping, Optional


@dataclass(frozen=True)
class LocalizationCandidate:
    label: str
    map_path: str
    pbstream_path: str


def _resolve_path(value: str, source_path: str | os.PathLike[str] | None) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    expanded = os.path.expanduser(text)
    if os.path.isabs(expanded):
        return expanded
    if source_path:
        base = Path(source_path)
        if base.suffix:
            base = base.parent
        return str(base / expanded)
    return expanded


def localization_candidates(
    navigation: Mapping[str, object],
    source_path: str | os.PathLike[str] | None,
    arena: str = "",
) -> list[LocalizationCandidate]:
    arenas = navigation.get("arenas", {})
    if not isinstance(arenas, Mapping):
        return []
    result: list[LocalizationCandidate] = []
    seen: set[tuple[str, str]] = set()
    selected = str(arena or "").strip().upper()
    labels = (selected,) if selected in {"A", "B"} else ("A", "B")
    for label in labels:
        raw = arenas.get(label, {})
        if not isinstance(raw, Mapping):
            continue
        map_path = _resolve_path(str(raw.get("map", "") or ""), source_path)
        pbstream_path = _resolve_path(str(raw.get("pbstream", "") or ""), source_path)
        key = (map_path, pbstream_path)
        if not map_path or not pbstream_path or key in seen:
            continue
        seen.add(key)
        result.append(LocalizationCandidate(label, map_path, pbstream_path))
    return result


@dataclass
class StartupLocalizationLauncher:
    launch_package: str = "at_nav2"
    launch_file: str = "startup_localization.launch.py"
    process: Optional[subprocess.Popen] = None

    def build_command(self, candidate: LocalizationCandidate) -> list[str]:
        return [
            "ros2",
            "launch",
            self.launch_package,
            self.launch_file,
            f"pbstream:={candidate.pbstream_path}",
        ]

    def start(self, candidate: LocalizationCandidate) -> None:
        self.shutdown()
        self.process = subprocess.Popen(  # noqa: S603
            self.build_command(candidate),
            start_new_session=True,
        )

    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def shutdown(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if self.process.poll() is None:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.process = None
