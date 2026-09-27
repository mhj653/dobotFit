from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget

from core.project_manager import ProjectManager


class ProjectPage(QWidget):
    save_requested = Signal()
    load_requested = Signal()

    def __init__(self, project_manager: ProjectManager) -> None:
        super().__init__()
        self.project_manager = project_manager
        layout = QVBoxLayout(self)
        title = QLabel("Project")
        title.setProperty("class", "title")
        path = QLabel(str(project_manager.project_path))
        path.setWordWrap(True)
        save = QPushButton("Save Project")
        load = QPushButton("Load Project")
        save.setProperty("class", "primary")
        save.clicked.connect(self.save_requested)
        load.clicked.connect(self.load_requested)
        layout.addWidget(title)
        layout.addWidget(path)
        layout.addWidget(save)
        layout.addWidget(load)
        layout.addStretch()

