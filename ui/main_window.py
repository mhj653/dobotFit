from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QDateTime, QSettings, Qt, QTimer
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QInputDialog,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from core.device_manager import DeviceManager
from core.logger import AppLogger
from core.models import LogEvent, Position, SequenceStep
from core.project_manager import ProjectManager
from core.sequence_engine import SequenceEngine
from ui.pages.io_page import IOPage
from ui.pages.project_workspace_page import ProjectWorkspacePage
from ui.pages.robot_page import RobotPage
from ui.pages.sequence_page import SequencePage
from ui.pages.settings_page import SettingsPage
from ui.pages.vision_page import VisionPage
from ui.widgets.editable_title import EditableTitleLabel


class MainWindow(QMainWindow):
    def __init__(self, devices: DeviceManager, project_manager: ProjectManager, sequence_engine: SequenceEngine) -> None:
        super().__init__()
        self.devices = devices
        self.project_manager = project_manager
        self.sequence_engine = sequence_engine
        self.logger = AppLogger()
        self.positions: list[Position] = []
        self.sequence: list[SequenceStep] = []
        self.vision_profiles: dict = {}
        self.calibration_profiles: dict = {}
        self.app_settings = QSettings("Smart Factory Solutions", "Robot Automation Studio")
        self.theme_name = str(self.app_settings.value("appearance/theme", "Light"))
        if self.theme_name not in {"Light", "Dark"}:
            self.theme_name = "Light"
        self.app_title = str(self.app_settings.value("appearance/title", "Dobot MG400")).strip() or "Dobot MG400"

        self.setWindowTitle(self.app_title)
        self._load_style()
        self._build()
        self._wire()
        self.load_project()
        self.logger.log("INFO", "System", "Application started")

    def _load_style(self) -> None:
        file_name = "app_dark.qss" if self.theme_name == "Dark" else "app.qss"
        path = Path(__file__).resolve().parent / "styles" / file_name
        self.setStyleSheet(path.read_text(encoding="utf-8"))

    def _build(self) -> None:
        root = QWidget()
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._sidebar())

        shell = QWidget()
        shell_layout = QVBoxLayout(shell)
        shell_layout.setContentsMargins(0, 0, 0, 0)
        shell_layout.setSpacing(0)
        shell_layout.addWidget(self._topbar())
        self.stack = QStackedWidget()
        self.stack.setContentsMargins(12, 12, 12, 12)
        self.robot_page = RobotPage(self.devices)
        self.vision_page = VisionPage(self.devices)
        self.io_page = IOPage(self.devices)
        self.sequence_page = SequencePage(self.sequence_engine, self.devices)
        self.project_page = ProjectWorkspacePage(self.project_manager, self.robot_page, self.sequence_page)
        self.settings_page = SettingsPage(self.devices, self.theme_name)
        self.pages = [
            ("Project", self.project_page),
            ("Vision", self.vision_page),
            ("I/O", self.io_page),
            ("Settings", self.settings_page),
        ]
        for _, page in self.pages:
            self.stack.addWidget(page)
        shell_layout.addWidget(self.stack, 1)
        root_layout.addWidget(shell, 1)
        self.setCentralWidget(root)

    def _sidebar(self) -> QWidget:
        side = QFrame()
        side.setObjectName("sidebar")
        side.setFixedWidth(190)
        layout = QVBoxLayout(side)
        layout.setContentsMargins(10, 14, 10, 14)
        brand = QLabel("DOBOT\nMG400")
        brand.setObjectName("brand")
        subtitle = QLabel("Smart Factory Solutions")
        subtitle.setObjectName("subtitle")
        layout.addWidget(brand)
        layout.addWidget(subtitle)
        layout.addSpacing(16)
        self.nav_buttons: list[QPushButton] = []
        for index, name in enumerate(["Project", "Vision", "I/O", "Settings"]):
            button = QPushButton(name)
            button.setObjectName("navButton")
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, i=index: self._set_page(i))
            self.nav_buttons.append(button)
            layout.addWidget(button)
        self.nav_buttons[0].setChecked(True)
        layout.addStretch()
        small = QLabel("Robot Automation\nStudio")
        small.setObjectName("subtitle")
        layout.addWidget(small)
        return side

    def _topbar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topbar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(14, 10, 14, 10)
        self.sim_button = QPushButton("SIMULATION")
        self.real_button = QPushButton("REAL ROBOT")
        self.sim_button.setProperty("class", "primary")
        self.sim_button.clicked.connect(lambda: self._set_mode("SIMULATION"))
        self.real_button.clicked.connect(lambda: self._set_mode("REAL"))
        self.title_label = EditableTitleLabel(self.app_title)
        self.title_label.setObjectName("topbarTitle")
        self.title_label.setToolTip("Double-click to rename the application title")
        self.title_label.double_clicked.connect(self._edit_app_title)
        self.connection = QLabel("MG400 Connected")
        self.connection.setProperty("class", "ok")
        self.alarm = QLabel("No Alarm")
        self.alarm.setProperty("class", "muted")
        self.clock = QLabel()
        layout.addWidget(self.sim_button)
        layout.addWidget(self.real_button)
        layout.addSpacing(12)
        layout.addWidget(self.title_label, 1)
        layout.addStretch()
        layout.addWidget(self.connection)
        layout.addSpacing(16)
        layout.addWidget(self.alarm)
        layout.addSpacing(16)
        layout.addWidget(self.clock)
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._tick_clock)
        self.clock_timer.start(1000)
        self._tick_clock()
        return bar

    def _wire(self) -> None:
        self.devices.state_changed.connect(self.refresh_all)
        self.devices.log_requested.connect(self.logger.log)
        self.logger.event_added.connect(self._on_log)
        self.robot_page.positions_changed.connect(self._sync_positions)
        self.sequence_page.sequence_changed.connect(self._sync_sequence)
        self.project_page.save_requested.connect(self.save_project)
        self.project_page.load_requested.connect(self.load_project)
        self.settings_page.theme_changed.connect(self._set_theme)
        self.vision_page.profiles_changed.connect(self._sync_profile_names)
        self.sequence_engine.step_finished.connect(lambda step: self.logger.log("INFO", "Sequence", f"Step {step.no} {step.status} ({step.elapsed_ms / 1000.0:.2f} s)"))

    def _edit_app_title(self) -> None:
        title, ok = QInputDialog.getText(self, "Edit Title", "Application title", text=self.app_title)
        if not ok:
            return
        title = title.strip()
        if not title:
            return
        self._set_app_title(title)
        self.logger.log("INFO", "System", f"Application title changed to {title}")

    def _set_app_title(self, title: str) -> None:
        self.app_title = title
        self.setWindowTitle(title)
        if hasattr(self, "title_label"):
            self.title_label.setText(title)
        self.app_settings.setValue("appearance/title", title)

    def _set_page(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        for i, button in enumerate(self.nav_buttons):
            button.setChecked(i == index)

    def _set_mode(self, mode: str) -> None:
        self.devices.set_mode("REAL" if mode == "REAL" else "SIMULATION")
        self.logger.log("INFO", "System", f"Mode changed to {self.devices.mode}")

    def _set_theme(self, theme: str) -> None:
        self.theme_name = theme if theme in {"Light", "Dark"} else "Light"
        self.app_settings.setValue("appearance/theme", self.theme_name)
        self._load_style()
        self.settings_page.set_theme(self.theme_name)
        self.logger.log("INFO", "System", f"Theme changed to {self.theme_name}")

    def _tick_clock(self) -> None:
        self.clock.setText(QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss"))

    def load_project(self) -> None:
        data = self.project_manager.load()
        self.settings_page.load_robot_config(data["robot"])
        self.positions = data["positions"]
        self.sequence = data["sequence"]
        self.devices.io.alias = data["io_mapping"]
        self.vision_profiles = data["vision"]
        self.calibration_profiles = data["calibration"]
        self._push_project_data()
        self.logger.log("INFO", "Project", f"Loaded {self.project_manager.project_path.name}")

    def save_project(self) -> None:
        self._sync_positions()
        self._sync_sequence()
        cfg = self.devices.real_robot_config
        robot_config = {
            "model": "MG400",
            "mode": self.devices.mode,
            "ip": cfg.ip,
            "dashboard_port": cfg.dashboard_port,
            "move_port": cfg.move_port,
            "feedback_port": cfg.feedback_port,
            "timeout_s": cfg.timeout_s,
        }
        self.vision_profiles, self.calibration_profiles = self.vision_page.project_profiles()
        self.project_manager.save(
            self.positions,
            self.sequence,
            self.devices.io.alias,
            robot_config,
            self.vision_profiles,
            self.calibration_profiles,
        )
        self._sync_profile_names()
        self.logger.log("INFO", "Project", f"Saved {self.project_manager.project_path.name}")

    def _push_project_data(self) -> None:
        self.vision_page.set_project_profiles(self.vision_profiles, self.calibration_profiles)
        self._sync_profile_names()
        self.robot_page.set_positions(self.positions)
        self.sequence_page.set_data(self.sequence, self.positions)
        self.refresh_all()

    def _sync_profile_names(self) -> None:
        self.vision_profiles, self.calibration_profiles = self.vision_page.project_profiles()
        vision_names, calibration_names = self.vision_page.profile_names()
        self.sequence_page.set_profile_names(vision_names, calibration_names)
        self.sequence_engine.set_profiles(self.vision_profiles, self.calibration_profiles)

    def _sync_positions(self) -> None:
        self.positions = self.robot_page.positions
        self.sequence_page.positions = self.positions

    def _sync_sequence(self) -> None:
        self.sequence = self.sequence_page.sequence

    def _on_log(self, event: LogEvent) -> None:
        _ = event

    def refresh_all(self) -> None:
        self.robot_page.refresh()
        self.io_page.refresh()
        if self.devices.mode == "REAL":
            connected = getattr(self.devices.robot, "dashboard", None) is not None and self.devices.robot.dashboard.connected
            self.connection.setText("REAL MG400 CONNECTED" if connected else "REAL MG400 NOT CONNECTED")
            self.connection.setProperty("class", "ok" if connected else "bad")
        else:
            self.connection.setText("MG400 Simulation Connected")
            self.connection.setProperty("class", "ok")
        self.alarm.setText(self.devices.last_alarm or "No Alarm")
        self.alarm.setProperty("class", "bad" if self.devices.last_alarm else "muted")
        self.connection.style().unpolish(self.connection)
        self.connection.style().polish(self.connection)
        self.alarm.style().unpolish(self.alarm)
        self.alarm.style().polish(self.alarm)
