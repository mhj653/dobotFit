from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QTabWidget, QVBoxLayout, QWidget

from core.project_manager import ProjectManager
from ui.pages.robot_page import RobotPage
from ui.pages.sequence_page import SequencePage


class ProjectWorkspacePage(QWidget):
    save_requested = Signal()
    load_requested = Signal()

    def __init__(
        self,
        project_manager: ProjectManager,
        robot_page: RobotPage,
        sequence_page: SequencePage,
    ) -> None:
        super().__init__()
        self.project_manager = project_manager
        self.robot_page = robot_page
        self.sequence_page = sequence_page

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._project_header())

        self.tabs = QTabWidget()
        self.tabs.addTab(self.robot_page, "Robot")
        self.tabs.addTab(self.sequence_page, "Sequence + Analysis")
        layout.addWidget(self.tabs, 1)

    def _project_header(self) -> QWidget:
        header = QFrame()
        header.setObjectName("panel")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(12, 10, 12, 10)

        title = QLabel("Project")
        title.setProperty("class", "title")
        path = QLabel(str(self.project_manager.project_path))
        path.setWordWrap(True)
        save = QPushButton("Save Project")
        load = QPushButton("Load Project")
        save.setProperty("class", "primary")
        save.clicked.connect(self.save_requested)
        load.clicked.connect(self.load_requested)

        layout.addWidget(title)
        layout.addWidget(path, 1)
        layout.addWidget(save)
        layout.addWidget(load)
        return header
