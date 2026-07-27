#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Application entry point for the PySide6 + QML teleoperation console."""

from __future__ import annotations

import os
import signal
import sys
from pathlib import Path

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QIcon
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtWidgets import QApplication

from backend import LogModel, TeleopBackend


def resource_path(*parts: str) -> Path:
    """Return a resource path in source and PyInstaller bundle modes."""
    bundle_root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return bundle_root.joinpath(*parts)


def set_windows_app_id() -> None:
    """Ensure the Windows taskbar uses this application's own icon/group."""
    if os.name != "nt":
        return
    try:
        import ctypes

        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "AgroTechSCAU.AtlasTeleop.1"
        )
    except Exception:
        # Cosmetic only; failure must never block teleoperation startup.
        pass


def main() -> int:
    set_windows_app_id()

    app = QApplication(sys.argv)
    app.setApplicationName("Atlas Teleop")
    app.setApplicationDisplayName("主从臂遥操作控制台")
    app.setOrganizationName("AgroTech-SCAU")
    app.setOrganizationDomain("agrotech-scau.local")

    icon_path = resource_path("assets", "app.png")
    if icon_path.exists():
        app.setWindowIcon(QIcon(str(icon_path)))

    # Keep Python signal handling responsive while the Qt event loop is active.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    heartbeat = QTimer()
    heartbeat.timeout.connect(lambda: None)
    heartbeat.start(200)

    log_model = LogModel()
    backend = TeleopBackend(log_model)

    engine = QQmlApplicationEngine()
    context = engine.rootContext()
    context.setContextProperty("backend", backend)
    context.setContextProperty("logModel", log_model)

    qml_path = resource_path("qml", "Main.qml")
    engine.load(QUrl.fromLocalFile(str(qml_path)))
    if not engine.rootObjects():
        return 1

    app.aboutToQuit.connect(backend.shutdown)
    QTimer.singleShot(250, backend.scanPorts)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
