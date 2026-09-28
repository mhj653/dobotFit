from __future__ import annotations

from PySide6.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QLabel, QPushButton, QVBoxLayout, QWidget

from core.device_manager import DeviceManager


class IOPage(QWidget):
    def __init__(self, devices: DeviceManager) -> None:
        super().__init__()
        self.devices = devices
        self.di_checks: dict[int, QCheckBox] = {}
        self.do_checks: dict[int, QCheckBox] = {}
        self.gripper_open_label = QLabel()
        self.gripper_close_label = QLabel()
        self.gripper_sensor_label = QLabel()
        layout = QGridLayout(self)
        layout.addWidget(self._io_box("Digital Input (DI)", True), 0, 0)
        layout.addWidget(self._io_box("Digital Output (DO)", False), 0, 1)
        layout.addWidget(self._gripper_box(), 0, 2)
        self.refresh()

    def _io_box(self, title: str, is_di: bool) -> QGroupBox:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        for channel in range(1, 9):
            check = QCheckBox(("DI" if is_di else "DO") + f" {channel:02d}")
            if is_di:
                check.stateChanged.connect(lambda state, ch=channel: self.devices.set_di(ch, state > 0))
                self.di_checks[channel] = check
            else:
                check.stateChanged.connect(lambda state, ch=channel: self.devices.set_do(ch, state > 0))
                self.do_checks[channel] = check
            layout.addWidget(check)
        layout.addStretch()
        return box

    def _gripper_box(self) -> QGroupBox:
        box = QGroupBox("Soft Gripper Kit")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Model: DOBOT Soft Gripper Kit 1"))
        layout.addWidget(QLabel("Fingers: 4-Finger"))
        layout.addWidget(self.gripper_open_label)
        layout.addWidget(self.gripper_close_label)
        layout.addWidget(self.gripper_sensor_label)
        open_button = QPushButton("Open")
        close_button = QPushButton("Close")
        open_button.clicked.connect(self.devices.gripper_open)
        close_button.clicked.connect(self.devices.gripper_close)
        layout.addWidget(open_button)
        layout.addWidget(close_button)
        layout.addStretch()
        return box

    def refresh(self) -> None:
        cfg = self.devices.gripper_io
        self.gripper_open_label.setText(f"OPEN: {cfg.open_label}")
        self.gripper_close_label.setText(f"CLOSE: {cfg.close_label}")
        self.gripper_sensor_label.setText(f"Sensor: {cfg.sensor_label}")
        for channel, check in self.di_checks.items():
            check.blockSignals(True)
            check.setChecked(self.devices.io.di[channel])
            check.setEnabled(self.devices.mode != "REAL")
            check.blockSignals(False)
        for channel, check in self.do_checks.items():
            check.blockSignals(True)
            check.setChecked(self.devices.io.do[channel])
            check.setEnabled(True)
            check.blockSignals(False)
