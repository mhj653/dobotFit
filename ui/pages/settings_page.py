from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from core.device_manager import DeviceManager
from core.tool_config import GripperIOConfig


class SettingsPage(QWidget):
    theme_changed = Signal(str)

    def __init__(self, devices: DeviceManager, current_theme: str = "Light") -> None:
        super().__init__()
        self.devices = devices
        self.current_theme = current_theme if current_theme in {"Light", "Dark"} else "Light"
        layout = QVBoxLayout(self)
        tabs = QTabWidget()
        tabs.addTab(self._robot_tab(), "Robot")
        tabs.addTab(self._end_effector_tab(), "End Effector")
        tabs.addTab(self._camera_tab(), "Camera")
        tabs.addTab(self._plc_tab(), "PLC")
        tabs.addTab(self._system_tab(), "System")
        layout.addWidget(tabs)

    def _robot_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        connection = QGroupBox("Connection")
        form = QFormLayout(connection)
        self.mode = QComboBox()
        self.mode.addItems(["SIMULATION", "REAL"])
        self.mode.currentTextChanged.connect(lambda text: self.devices.set_mode("REAL" if text == "REAL" else "SIMULATION"))
        form.addRow("Mode", self.mode)
        self.ip_address = QLineEdit(self.devices.real_robot_config.ip)
        self.dashboard_port = self._port_spin(self.devices.real_robot_config.dashboard_port)
        self.move_port = self._port_spin(self.devices.real_robot_config.move_port)
        self.feedback_port = self._port_spin(self.devices.real_robot_config.feedback_port)
        self.timeout_s = QDoubleSpinBox()
        self.timeout_s.setRange(0.1, 30.0)
        self.timeout_s.setSingleStep(0.1)
        self.timeout_s.setValue(self.devices.real_robot_config.timeout_s)
        self.timeout_s.setSuffix(" s")
        form.addRow("IP Address", self.ip_address)
        form.addRow("Dashboard Port", self.dashboard_port)
        form.addRow("Move Port", self.move_port)
        form.addRow("Feedback Port", self.feedback_port)
        form.addRow("Timeout", self.timeout_s)
        apply = QPushButton("Apply Connection Settings")
        apply.clicked.connect(self.apply_connection_settings)
        connect = QPushButton("Connect")
        connect.clicked.connect(self.devices.connect_all)
        form.addRow(apply)
        form.addRow(connect)
        safety = QGroupBox("Safety & Limit")
        sform = QFormLayout(safety)
        sform.addRow("J1", QLabel("-160..160 deg"))
        sform.addRow("J2", QLabel("-160..160 deg"))
        sform.addRow("J3", QLabel("-160..160 deg"))
        sform.addRow("J4", QLabel("-360..360 deg"))
        sform.addRow("X/Y", QLabel("-440..440 mm"))
        sform.addRow("Z", QLabel("-5..400 mm"))
        sform.addRow(QCheckBox("Enable software limit"))
        layout.addWidget(connection)
        layout.addWidget(safety)
        layout.addStretch()
        return page

    def _port_spin(self, value: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(1, 65535)
        spin.setValue(value)
        return spin

    def apply_connection_settings(self) -> None:
        self.devices.configure_real_robot(
            self.ip_address.text().strip() or "192.168.1.6",
            self.dashboard_port.value(),
            self.move_port.value(),
            self.feedback_port.value(),
            self.timeout_s.value(),
        )

    def load_robot_config(self, config: dict) -> None:
        self.ip_address.setText(str(config.get("ip", "192.168.1.6")))
        self.dashboard_port.setValue(int(config.get("dashboard_port", 29999)))
        self.move_port.setValue(int(config.get("move_port", 30003)))
        self.feedback_port.setValue(int(config.get("feedback_port", 30004)))
        self.timeout_s.setValue(float(config.get("timeout_s", 1.5)))
        self.apply_connection_settings()

    def _end_effector_tab(self) -> QWidget:
        page = QWidget()
        layout = QFormLayout(page)
        layout.addRow("Tool", QLabel("DOBOT Soft Gripper Kit"))
        self.gripper_open_output = self._channel_spin(self.devices.gripper_io.open_output)
        self.gripper_close_output = self._channel_spin(self.devices.gripper_io.close_output)
        self.gripper_sensor_input = self._channel_spin(self.devices.gripper_io.sensor_input)
        layout.addRow("OPEN Output", self.gripper_open_output)
        layout.addRow("CLOSE Output", self.gripper_close_output)
        layout.addRow("Sensor Input", self.gripper_sensor_input)
        apply = QPushButton("Apply Gripper I/O")
        apply.clicked.connect(self.apply_gripper_settings)
        layout.addRow(apply)
        return page

    def _channel_spin(self, value: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(1, 8)
        spin.setValue(value)
        return spin

    def apply_gripper_settings(self) -> None:
        self.devices.configure_gripper_io(
            self.gripper_open_output.value(),
            self.gripper_close_output.value(),
            self.gripper_sensor_input.value(),
        )

    def load_gripper_config(self, config: GripperIOConfig) -> None:
        if not hasattr(self, "gripper_open_output"):
            return
        self.gripper_open_output.setValue(config.open_output)
        self.gripper_close_output.setValue(config.close_output)
        self.gripper_sensor_input.setValue(config.sensor_input)

    def _camera_tab(self) -> QWidget:
        page = QWidget()
        layout = QFormLayout(page)
        layout.addRow("Vision Camera", QLabel("Intel RealSense D405"))
        layout.addRow("Streams", QLabel("RGB + Depth aligned to RGB"))
        layout.addRow("Detection", QLabel("OpenCV Blob center with ROI and depth locate"))
        layout.addRow("Calibration", QLabel("Camera-to-robot matrix in Vision tab"))
        return page

    def _plc_tab(self) -> QWidget:
        page = QWidget()
        layout = QFormLayout(page)
        layout.addRow("PLC", QLabel("MockPLC now, Mitsubishi Q-Series later"))
        layout.addRow("Protocol", QLabel("MC Protocol ASCII / SLMP planned"))
        return page

    def _system_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        appearance = QGroupBox("Appearance")
        form = QFormLayout(appearance)
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["Light", "Dark"])
        self.theme_combo.setCurrentText(self.current_theme)
        self.theme_combo.currentTextChanged.connect(self.theme_changed.emit)
        form.addRow("Theme", self.theme_combo)
        layout.addWidget(appearance)
        layout.addWidget(QLabel("Logs, project path, and packaging settings are stored per project."))
        layout.addStretch()
        return page

    def set_theme(self, theme: str) -> None:
        if not hasattr(self, "theme_combo"):
            self.current_theme = theme
            return
        self.theme_combo.blockSignals(True)
        self.theme_combo.setCurrentText(theme if theme in {"Light", "Dark"} else "Light")
        self.theme_combo.blockSignals(False)
