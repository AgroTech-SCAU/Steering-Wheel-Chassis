from __future__ import annotations

import os
import signal
import subprocess
from dataclasses import dataclass
from typing import Optional


@dataclass
class Nav2StackLauncher:
    launch_package: str = "at_nav2"
    launch_file: str = "at_nav.launch.py"
    params_file: str = ""
    cmd_vel_output: str = "/atlas/navigation/cmd_vel"
    process: Optional[subprocess.Popen] = None

    def build_command(self, map_path: str, pbstream_path: str) -> list[str]:
        return [
            "ros2",
            "launch",
            self.launch_package,
            self.launch_file,
            f"map:={map_path}",
            f"pbstream:={pbstream_path}",
            f"params_file:={self.params_file}",
            f"cmd_vel_output:={self.cmd_vel_output}",
        ]

    def start(self, map_path: str, pbstream_path: str) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        self.process = subprocess.Popen(  # noqa: S603
            self.build_command(map_path, pbstream_path),
            start_new_session=True,
        )

    def shutdown(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
