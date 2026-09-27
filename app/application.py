from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from core.device_manager import DeviceManager
from core.project_manager import ProjectManager
from core.sequence_engine import SequenceEngine
from ui.main_window import MainWindow


def run() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Automation Studio")
    app.setOrganizationName("Smart Factory Solutions")

    root = Path(__file__).resolve().parents[1]
    project_manager = ProjectManager(root / "projects" / "TestProject")
    device_manager = DeviceManager()
    sequence_engine = SequenceEngine(device_manager)

    window = MainWindow(device_manager, project_manager, sequence_engine)
    screen = app.primaryScreen()
    if screen is not None:
        window.setGeometry(screen.availableGeometry())
    window.showMaximized()
    return app.exec()
