from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from core.device_manager import DeviceManager
from core.models import LogEvent
from ui.widgets.cards import MetricCard, card


class DashboardPage(QWidget):
    def __init__(self, devices: DeviceManager) -> None:
        super().__init__()
        self.devices = devices
        layout = QVBoxLayout(self)

        grid = QGridLayout()
        self.status = MetricCard("System Status", "READY", "Simulation is ready")
        self.robot = MetricCard("Robot", "MG400", "Connected")
        self.gripper = MetricCard("Soft Gripper", "Simulation", "Connected")
        self.camera2d = MetricCard("Vision Camera", "RealSense D405", "Ready")
        self.camera3d = MetricCard("Depth", "RGB-D", "Ready")
        self.plc = MetricCard("PLC", "MockPLC", "Connected")
        for index, widget in enumerate([self.status, self.robot, self.gripper, self.camera2d, self.camera3d, self.plc]):
            grid.addWidget(widget, index // 3, index % 3)
        layout.addLayout(grid)

        log_card = card()
        log_layout = QVBoxLayout(log_card)
        title = QLabel("Recent Log")
        title.setProperty("class", "title")
        self.log_table = QTableWidget(0, 4)
        self.log_table.setHorizontalHeaderLabels(["Time", "Level", "Source", "Message"])
        self.log_table.horizontalHeader().setStretchLastSection(True)
        log_layout.addWidget(title)
        log_layout.addWidget(self.log_table)
        layout.addWidget(log_card, 1)

    def refresh(self) -> None:
        pose = self.devices.robot_pose()
        self.status.value.setText("ALARM" if self.devices.last_alarm else "READY")
        self.status.value.setProperty("class", "bad" if self.devices.last_alarm else "ok")
        self.status.subtitle.setText(self.devices.last_alarm or "System is ready")
        self.robot.subtitle.setText(f"{self.devices.mode} / X {pose.x:.1f} Y {pose.y:.1f} Z {pose.z:.1f}")
        self.gripper.subtitle.setText("Closed" if getattr(self.devices.gripper, "closed", False) else "Open")
        connected = bool(getattr(self.devices.camera3d, "connected", False))
        self.camera2d.subtitle.setText("Connected" if connected else "Not connected")
        self.camera3d.subtitle.setText("Depth ready" if connected else "Connect in Vision")

    def add_log(self, event: LogEvent) -> None:
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        for col, value in enumerate(event.to_row()):
            self.log_table.setItem(row, col, QTableWidgetItem(str(value)))
        self.log_table.scrollToBottom()
