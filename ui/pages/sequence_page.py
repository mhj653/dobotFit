from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from core.device_manager import DeviceManager
from core.models import Position, SequenceStep
from core.sequence_engine import SequenceEngine
from core.vision_profiles import DEFAULT_CALIBRATION_PROFILE, DEFAULT_VISION_PROFILE
from ui.widgets.robot_canvas import RobotCanvas
from ui.widgets.tcp_monitor import TcpMonitorWidget


DEVICE_COMMANDS = {
    "Robot": ["MoveJ", "MoveL", "Jog"],
    "Gripper": ["Open", "Close"],
    "Wait": ["Time", "DI"],
    "I/O": ["SetDO"],
    "Vision": ["Detect"],
    "PLC": ["SetOutput"],
    "Comment": ["Comment"],
}

STEP_TEMPLATES = {
    "Pick": [
        ("Robot", "MoveJ", "PICK_APPROACH"),
        ("Robot", "MoveL", "PICK"),
        ("Gripper", "Close", ""),
        ("Wait", "DI", "1"),
        ("Robot", "MoveL", "PICK_APPROACH"),
    ],
    "Place": [
        ("Robot", "MoveJ", "PLACE"),
        ("Gripper", "Open", ""),
        ("Wait", "Time", "0.30"),
    ],
    "Vision Pick": [
        ("Robot", "MoveJ", "CAMERA"),
        ("Vision", "Detect", "ScrewHole"),
        ("Robot", "MoveL", "VISION_XYZ"),
        ("Gripper", "Close", ""),
    ],
    "Inspect Then Output": [
        ("Vision", "Detect", "D405_LOCATE"),
        ("PLC", "SetOutput", "COMPLETE=ON"),
        ("I/O", "SetDO", "DO03=ON"),
    ],
}


class SequencePage(QWidget):
    sequence_changed = Signal()

    def __init__(self, engine: SequenceEngine, devices: DeviceManager) -> None:
        super().__init__()
        self.engine = engine
        self.devices = devices
        self.sequence: list[SequenceStep] = []
        self.positions: list[Position] = []
        self.vision_profile_names: list[str] = [DEFAULT_VISION_PROFILE]
        self.calibration_profile_names: list[str] = [DEFAULT_CALIBRATION_PROFILE]
        self.current_index = 0
        self._syncing_editor = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._auto_tick)

        layout = QVBoxLayout(self)
        layout.addLayout(self._build_toolbar())

        work_area = QHBoxLayout()
        sequence_panel = QWidget()
        sequence_panel.setLayout(self._build_sequence_area())
        work_area.addWidget(sequence_panel, 5)
        work_area.addWidget(self._build_side_area(), 2)
        layout.addLayout(work_area, 3)

        bottom = QHBoxLayout()
        self.variables = QLabel("Variables: -")
        self.variables.setWordWrap(False)
        self.variables.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.progress = QLabel("Previous: - | Current: - | Next: -")
        bottom.addWidget(self.progress, 2)
        bottom.addWidget(self.variables, 3)
        layout.addLayout(bottom)

        self.engine.variables_changed.connect(self._set_variables)
        self.engine.step_finished.connect(lambda step: self._on_step_finished())
        self.devices.state_changed.connect(self.refresh_preview)
        self.devices.packet_logged.connect(self._append_packet_log)
        self.refresh_preview()
        self._set_editor_enabled(False)

    def _build_toolbar(self) -> QHBoxLayout:
        toolbar = QHBoxLayout()
        add = QPushButton("Add Step")
        add_template = QPushButton("Add Template")
        duplicate = QPushButton("Duplicate")
        remove = QPushButton("Remove")
        up = QPushButton("Up")
        down = QPushButton("Down")
        reset = QPushButton("Reset")
        validate = QPushButton("Validate")
        run = QPushButton("Run")
        step = QPushButton("Step")
        stop = QPushButton("Stop")
        self.template_combo = QComboBox()
        self.template_combo.addItems(list(STEP_TEMPLATES.keys()))
        run.setProperty("class", "success")
        step.setProperty("class", "primary")
        add.clicked.connect(self._add_step)
        add_template.clicked.connect(self._add_template)
        duplicate.clicked.connect(self._duplicate_selected)
        remove.clicked.connect(self._remove_step)
        up.clicked.connect(lambda: self._move_selected(-1))
        down.clicked.connect(lambda: self._move_selected(1))
        reset.clicked.connect(self.reset)
        validate.clicked.connect(self.validate_sequence)
        run.clicked.connect(self.run_auto)
        step.clicked.connect(self.run_step)
        stop.clicked.connect(self.stop)
        toolbar.addWidget(QLabel("Template"))
        toolbar.addWidget(self.template_combo)
        for widget in [add_template, add, duplicate, remove, up, down, reset, validate, run, step, stop]:
            toolbar.addWidget(widget)
        toolbar.addStretch()
        return toolbar

    def _build_sequence_area(self) -> QVBoxLayout:
        area = QVBoxLayout()
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(["On", "Device", "Command", "Setting", "Condition", "Speed", "Status", "Time s"])
        header = self.table.horizontalHeader()
        header.setStretchLastSection(False)
        for column, width in enumerate([44, 78, 84, 320, 110, 66, 76, 64]):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.table.setColumnWidth(column, width)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        self.table.setWordWrap(False)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.currentCellChanged.connect(lambda *_: self._load_selected_step())
        area.addWidget(self.table, 5)
        area.addWidget(self._build_step_editor(), 2)
        return area

    def _build_side_area(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(360)
        side = QVBoxLayout(panel)
        side.setContentsMargins(0, 0, 0, 0)
        preview_box = QGroupBox("Simulation Preview")
        preview_layout = QVBoxLayout(preview_box)
        self.preview_canvas = RobotCanvas()
        self.preview_canvas.setMinimumHeight(250)
        preview_layout.addWidget(self.preview_canvas)
        self.tcp_monitor = TcpMonitorWidget()
        side.addWidget(preview_box, 3)
        side.addWidget(self.tcp_monitor, 2)
        return panel

    def _build_step_editor(self) -> QGroupBox:
        editor = QGroupBox("Selected Step Settings")
        editor.setMaximumHeight(320)
        form = QFormLayout(editor)
        self.enabled_check = QCheckBox("Run this step")
        self.enabled_check.setChecked(True)
        self.device_combo = QComboBox()
        self.device_combo.addItems(list(DEVICE_COMMANDS.keys()))
        self.command_combo = QComboBox()
        self.condition_combo = QComboBox()
        self.condition_combo.setEditable(True)
        self.condition_combo.addItems(["", "part_ok", "not part_ok", "grip_ok", "DI01=ON", "DI01=OFF", "count>0"])
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1.0, 100.0)
        self.speed_spin.setValue(20.0)
        self.speed_spin.setSuffix(" %")
        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(1.0, 100.0)
        self.accel_spin.setValue(50.0)
        self.accel_spin.setSuffix(" %")
        self.enabled_check.toggled.connect(self._apply_editor_to_step)
        self.device_combo.currentTextChanged.connect(self._on_device_changed)
        self.command_combo.currentTextChanged.connect(self._on_command_changed)
        self.condition_combo.currentTextChanged.connect(self._apply_editor_to_step)
        self.speed_spin.valueChanged.connect(self._apply_editor_to_step)
        self.accel_spin.valueChanged.connect(self._apply_editor_to_step)
        form.addRow("Enabled", self.enabled_check)
        form.addRow("Device", self.device_combo)
        form.addRow("Command", self.command_combo)
        form.addRow("Condition", self.condition_combo)
        form.addRow("Speed", self.speed_spin)
        form.addRow("Accel", self.accel_spin)

        self.target_stack = QStackedWidget()
        self.position_combo = QComboBox()
        self.position_combo.setEditable(False)
        self.target_stack.addWidget(self.position_combo)

        jog_panel = QWidget()
        jog_layout = QHBoxLayout(jog_panel)
        jog_layout.setContentsMargins(0, 0, 0, 0)
        self.jog_axis = QComboBox()
        self.jog_axis.addItems(["X", "Y", "Z", "R"])
        self.jog_direction = QComboBox()
        self.jog_direction.addItems(["+", "-"])
        self.jog_amount = QDoubleSpinBox()
        self.jog_amount.setRange(0.1, 200.0)
        self.jog_amount.setValue(10.0)
        self.jog_amount.setSuffix(" mm/deg")
        jog_layout.addWidget(self.jog_axis)
        jog_layout.addWidget(self.jog_direction)
        jog_layout.addWidget(self.jog_amount)
        self.target_stack.addWidget(jog_panel)

        self.time_seconds = QDoubleSpinBox()
        self.time_seconds.setRange(0.0, 300.0)
        self.time_seconds.setDecimals(2)
        self.time_seconds.setSingleStep(0.10)
        self.time_seconds.setValue(0.30)
        self.time_seconds.setSuffix(" s")
        self.target_stack.addWidget(self.time_seconds)

        self.di_channel = QComboBox()
        self.di_channel.addItems([f"DI{idx:02d}" for idx in range(1, 9)])
        self.target_stack.addWidget(self.di_channel)

        do_panel = QWidget()
        do_layout = QHBoxLayout(do_panel)
        do_layout.setContentsMargins(0, 0, 0, 0)
        self.do_channel = QComboBox()
        self.do_channel.addItems([f"DO{idx:02d}" for idx in range(1, 9)])
        self.do_state = QComboBox()
        self.do_state.addItems(["ON", "OFF"])
        do_layout.addWidget(self.do_channel)
        do_layout.addWidget(self.do_state)
        self.target_stack.addWidget(do_panel)

        self.vision_target = QComboBox()
        self.vision_target.setEditable(True)
        self.vision_target.addItems(["D405_LOCATE", "Pattern", "Blob", "Presence"])
        vision_panel = QWidget()
        vision_layout = QGridLayout(vision_panel)
        vision_layout.setContentsMargins(0, 0, 0, 0)
        vision_layout.setHorizontalSpacing(6)
        vision_layout.setVerticalSpacing(4)
        self.vision_profile_combo = QComboBox()
        self.calibration_profile_combo = QComboBox()
        for combo in [self.vision_profile_combo, self.calibration_profile_combo]:
            combo.setMinimumWidth(150)
        self._refresh_profile_combos()
        vision_layout.addWidget(QLabel("Target"), 0, 0)
        vision_layout.addWidget(self.vision_target, 0, 1)
        vision_layout.addWidget(QLabel("Vision"), 1, 0)
        vision_layout.addWidget(self.vision_profile_combo, 1, 1)
        vision_layout.addWidget(QLabel("Calibration"), 2, 0)
        vision_layout.addWidget(self.calibration_profile_combo, 2, 1)
        vision_layout.setColumnStretch(1, 1)
        self.target_stack.addWidget(vision_panel)

        self.plc_target = QLineEdit("COMPLETE=ON")
        self.target_stack.addWidget(self.plc_target)

        self.comment_target = QLineEdit()
        self.target_stack.addWidget(self.comment_target)

        self.no_target = QLabel("No parameter")
        self.target_stack.addWidget(self.no_target)
        form.addRow("Setting", self.target_stack)

        for widget in [
            self.position_combo,
            self.jog_axis,
            self.jog_direction,
            self.jog_amount,
            self.time_seconds,
            self.di_channel,
            self.do_channel,
            self.do_state,
            self.vision_target,
            self.vision_profile_combo,
            self.calibration_profile_combo,
            self.plc_target,
            self.comment_target,
        ]:
            if hasattr(widget, "currentTextChanged"):
                widget.currentTextChanged.connect(self._apply_editor_to_step)
            if hasattr(widget, "valueChanged"):
                widget.valueChanged.connect(self._apply_editor_to_step)
            if hasattr(widget, "textChanged"):
                widget.textChanged.connect(self._apply_editor_to_step)
        return editor

    def set_data(self, sequence: list[SequenceStep], positions: list[Position]) -> None:
        self.sequence = sequence
        self.positions = positions
        self.current_index = 0
        self._refresh_position_targets()
        self.refresh_table()
        self._set_variables(self.engine.variables)
        self.refresh_preview()

    def set_profile_names(self, vision_profiles: list[str], calibration_profiles: list[str]) -> None:
        self.vision_profile_names = vision_profiles or [DEFAULT_VISION_PROFILE]
        self.calibration_profile_names = calibration_profiles or [DEFAULT_CALIBRATION_PROFILE]
        self._refresh_profile_combos()
        self._load_selected_step()

    def refresh_table(self) -> None:
        selected = self.table.currentRow()
        self.table.blockSignals(True)
        self.table.setRowCount(0)
        for index, step in enumerate(self.sequence, start=1):
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setVerticalHeaderItem(row, QTableWidgetItem(str(index)))
            values = [
                "Yes" if step.enabled else "No",
                step.device,
                step.command,
                self._step_setting_text(step),
                step.condition or "Always",
                f"{step.speed:.0f}/{step.acceleration:.0f}" if step.device == "Robot" else "-",
                step.status,
                self._elapsed_seconds_text(step.elapsed_ms),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                item.setToolTip(str(value))
                self.table.setItem(row, col, item)
        self.table.blockSignals(False)
        if self.sequence:
            row = min(max(selected, 0), len(self.sequence) - 1)
            self.table.selectRow(row)
            self._load_selected_step()
        else:
            self._set_editor_enabled(False)
        self._update_progress_label()

    def run_step(self) -> None:
        if not self.sequence:
            return
        if self.current_index >= len(self.sequence):
            self.current_index = 0
        self.engine.execute_step(self.sequence[self.current_index], self.positions)
        self.current_index += 1
        self.refresh_table()
        self.refresh_preview()
        self.sequence_changed.emit()

    def run_auto(self) -> None:
        if not self.sequence:
            return
        issues = self.engine.validate_sequence(self.sequence, self.positions)
        if issues:
            self.progress.setText(f"Validation: {len(issues)} issue(s) - {issues[0]}")
            return
        self.current_index = 0
        for step in self.sequence:
            step.status = "Ready"
        self.timer.start(450)
        self.refresh_table()
        self.refresh_preview()

    def stop(self) -> None:
        self.timer.stop()

    def reset(self) -> None:
        self.timer.stop()
        self.current_index = 0
        for step in self.sequence:
            step.status = "Ready"
            step.elapsed_ms = 0.0
        self.sequence_changed.emit()
        self.refresh_table()

    def _auto_tick(self) -> None:
        if self.current_index >= len(self.sequence):
            self.timer.stop()
            return
        self.run_step()

    def _add_step(self) -> None:
        device = self.device_combo.currentText() if self.device_combo.isEnabled() else "Robot"
        command = DEVICE_COMMANDS[device][0]
        target = self._default_target(device, command)
        self.sequence.append(SequenceStep(len(self.sequence) + 1, device, command, target))
        self._renumber()
        self.sequence_changed.emit()
        self.refresh_table()
        self.table.selectRow(len(self.sequence) - 1)

    def _duplicate_selected(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.sequence):
            return
        step = self.sequence[row]
        self.sequence.insert(
            row + 1,
            SequenceStep(
                0,
                step.device,
                step.command,
                step.target,
                step.comment,
                step.enabled,
                step.condition,
                step.speed,
                step.acceleration,
                step.status,
                step.elapsed_ms,
                step.vision_profile,
                step.calibration_profile,
            ),
        )
        self._renumber()
        self.sequence_changed.emit()
        self.refresh_table()
        self.table.selectRow(row + 1)

    def _add_template(self) -> None:
        template = STEP_TEMPLATES.get(self.template_combo.currentText(), [])
        start_row = len(self.sequence)
        for device, command, target in template:
            self.sequence.append(
                SequenceStep(
                    len(self.sequence) + 1,
                    device,
                    command,
                    target if target else self._default_target(device, command),
                )
            )
        self._renumber()
        self.sequence_changed.emit()
        self.refresh_table()
        if self.sequence:
            self.table.selectRow(start_row)

    def _remove_step(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.sequence):
            return
        del self.sequence[row]
        self._renumber()
        self.sequence_changed.emit()
        self.refresh_table()

    def _move_selected(self, direction: int) -> None:
        row = self.table.currentRow()
        target = row + direction
        if row < 0 or target < 0 or target >= len(self.sequence):
            return
        self.sequence[row], self.sequence[target] = self.sequence[target], self.sequence[row]
        self._renumber()
        self.current_index = target
        self.sequence_changed.emit()
        self.refresh_table()
        self.table.selectRow(target)

    def _renumber(self) -> None:
        for index, step in enumerate(self.sequence, start=1):
            step.no = index

    def _load_selected_step(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.sequence):
            self._set_editor_enabled(False)
            return
        self._set_editor_enabled(True)
        step = self.sequence[row]
        self._syncing_editor = True
        self.enabled_check.setChecked(step.enabled)
        self.device_combo.setCurrentText(step.device if step.device in DEVICE_COMMANDS else "Comment")
        self._populate_commands(self.device_combo.currentText(), step.command)
        self.condition_combo.setCurrentText(step.condition)
        self.speed_spin.setValue(step.speed)
        self.accel_spin.setValue(step.acceleration)
        self._load_target(step)
        self._sync_motion_controls()
        self._syncing_editor = False

    def _set_editor_enabled(self, enabled: bool) -> None:
        for widget in [
            self.enabled_check,
            self.device_combo,
            self.command_combo,
            self.condition_combo,
            self.speed_spin,
            self.accel_spin,
            self.target_stack,
        ]:
            widget.setEnabled(enabled)

    def _on_device_changed(self, device: str) -> None:
        self._populate_commands(device)
        self._on_command_changed(self.command_combo.currentText())
        self._apply_editor_to_step()

    def _on_command_changed(self, command: str) -> None:
        self.target_stack.setCurrentIndex(self._target_page_index(self.device_combo.currentText(), command))
        self._sync_motion_controls()
        self._apply_editor_to_step()

    def _populate_commands(self, device: str, preferred: str | None = None) -> None:
        self.command_combo.blockSignals(True)
        self.command_combo.clear()
        commands = DEVICE_COMMANDS.get(device, ["Comment"])
        self.command_combo.addItems(commands)
        if preferred in commands:
            self.command_combo.setCurrentText(preferred)
        self.command_combo.blockSignals(False)

    def _refresh_position_targets(self) -> None:
        current = self.position_combo.currentText()
        self.position_combo.blockSignals(True)
        self.position_combo.clear()
        self.position_combo.addItems([position.name for position in self.positions])
        self.position_combo.addItem("VISION_XYZ")
        if current:
            self.position_combo.setCurrentText(current)
        self.position_combo.blockSignals(False)

    def _load_target(self, step: SequenceStep) -> None:
        page = self._target_page_index(self.device_combo.currentText(), self.command_combo.currentText())
        self.target_stack.setCurrentIndex(page)
        target = step.target.strip()
        if page == 0:
            self.position_combo.setCurrentText(target or self._default_position())
        elif page == 1:
            axis = target[0].upper() if target else "X"
            self.jog_axis.setCurrentText(axis if axis in {"X", "Y", "Z", "R"} else "X")
            self.jog_direction.setCurrentText("-" if "-" in target else "+")
            amount = "".join(ch for ch in target[1:] if ch.isdigit() or ch == ".")
            self.jog_amount.setValue(float(amount or 10.0))
        elif page == 2:
            self.time_seconds.setValue(self._wait_target_to_seconds(target or "0.30"))
        elif page == 3:
            index = int(float(target or 1))
            self.di_channel.setCurrentText(f"DI{index:02d}")
        elif page == 4:
            channel, state = self._split_assignment(target or "DO01=ON")
            self.do_channel.setCurrentText(channel if channel.startswith("DO") else "DO01")
            self.do_state.setCurrentText("OFF" if state.upper() in {"0", "OFF", "FALSE"} else "ON")
        elif page == 5:
            self.vision_target.setCurrentText(target or "D405_LOCATE")
            self._set_combo_text(self.vision_profile_combo, step.vision_profile or self._default_vision_profile())
            self._set_combo_text(self.calibration_profile_combo, step.calibration_profile or self._default_calibration_profile())
        elif page == 6:
            self.plc_target.setText(target or "COMPLETE=ON")
        elif page == 7:
            self.comment_target.setText(target)

    def _apply_editor_to_step(self, *args) -> None:
        if self._syncing_editor:
            return
        row = self.table.currentRow()
        if row < 0 or row >= len(self.sequence):
            return
        step = self.sequence[row]
        step.enabled = self.enabled_check.isChecked()
        step.device = self.device_combo.currentText()
        step.command = self.command_combo.currentText()
        step.target = self._target_from_editor()
        step.condition = self.condition_combo.currentText().strip()
        step.speed = self.speed_spin.value()
        step.acceleration = self.accel_spin.value()
        if step.device == "Vision":
            step.vision_profile = self.vision_profile_combo.currentText().strip()
            step.calibration_profile = self.calibration_profile_combo.currentText().strip()
        step.status = "Ready"
        step.elapsed_ms = 0.0
        self._renumber()
        self.sequence_changed.emit()
        self.refresh_table()
        self.table.selectRow(row)

    def _target_from_editor(self) -> str:
        page = self.target_stack.currentIndex()
        if page == 0:
            return self.position_combo.currentText() or self._default_position()
        if page == 1:
            return f"{self.jog_axis.currentText()}{self.jog_direction.currentText()}{self.jog_amount.value():g}"
        if page == 2:
            return f"{self.time_seconds.value():.2f}"
        if page == 3:
            return str(self.di_channel.currentIndex() + 1)
        if page == 4:
            return f"{self.do_channel.currentText()}={self.do_state.currentText()}"
        if page == 5:
            return self.vision_target.currentText().strip() or "D405_LOCATE"
        if page == 6:
            return self.plc_target.text().strip() or "COMPLETE=ON"
        if page == 7:
            return self.comment_target.text().strip()
        return ""

    def _target_page_index(self, device: str, command: str) -> int:
        if device == "Robot" and command in {"MoveJ", "MoveL"}:
            return 0
        if device == "Robot" and command == "Jog":
            return 1
        if device == "Wait" and command == "Time":
            return 2
        if device == "Wait" and command == "DI":
            return 3
        if device == "I/O":
            return 4
        if device == "Vision":
            return 5
        if device == "PLC":
            return 6
        if device == "Comment":
            return 7
        return 8

    def _default_target(self, device: str, command: str) -> str:
        page = self._target_page_index(device, command)
        defaults = {
            0: self._default_position(),
            1: "X+10",
            2: "0.30",
            3: "1",
            4: "DO01=ON",
            5: "D405_LOCATE",
            6: "COMPLETE=ON",
            7: "",
            8: "",
        }
        return defaults[page]

    def _refresh_profile_combos(self) -> None:
        if not hasattr(self, "vision_profile_combo"):
            return
        current_vision = self.vision_profile_combo.currentText()
        current_calibration = self.calibration_profile_combo.currentText()
        for combo, names, current in [
            (self.vision_profile_combo, self.vision_profile_names, current_vision),
            (self.calibration_profile_combo, self.calibration_profile_names, current_calibration),
        ]:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(names)
            if current in names:
                combo.setCurrentText(current)
            combo.blockSignals(False)

    def _set_combo_text(self, combo: QComboBox, text: str) -> None:
        if text and combo.findText(text) < 0:
            combo.addItem(text)
        combo.setCurrentText(text)

    def _default_vision_profile(self) -> str:
        return self.vision_profile_names[0] if self.vision_profile_names else DEFAULT_VISION_PROFILE

    def _default_calibration_profile(self) -> str:
        return self.calibration_profile_names[0] if self.calibration_profile_names else DEFAULT_CALIBRATION_PROFILE

    def _default_position(self) -> str:
        return self.positions[0].name if self.positions else "HOME"

    def _split_assignment(self, target: str) -> tuple[str, str]:
        if "=" not in target:
            return target.strip().upper() or "DO01", "ON"
        left, right = target.split("=", 1)
        return left.strip().upper(), right.strip().upper()

    def _sync_motion_controls(self) -> None:
        is_robot_move = self.device_combo.currentText() == "Robot" and self.command_combo.currentText() in {"MoveJ", "MoveL"}
        self.speed_spin.setEnabled(self.device_combo.isEnabled() and is_robot_move)
        self.accel_spin.setEnabled(self.device_combo.isEnabled() and is_robot_move)

    def validate_sequence(self) -> None:
        issues = self.engine.validate_sequence(self.sequence, self.positions)
        if issues:
            self.progress.setText(f"Validation: {len(issues)} issue(s) - {issues[0]}")
        else:
            self.progress.setText("Validation: OK")

    def _update_progress_label(self) -> None:
        prev = self.sequence[self.current_index - 1].no if 0 <= self.current_index - 1 < len(self.sequence) else "-"
        current = self.sequence[self.current_index].no if 0 <= self.current_index < len(self.sequence) else "-"
        next_step = self.sequence[self.current_index + 1].no if 0 <= self.current_index + 1 < len(self.sequence) else "-"
        self.progress.setText(f"Previous: {prev} | Current: {current} | Next: {next_step}")

    def _set_variables(self, variables: dict) -> None:
        full = ", ".join(f"{key}={value}" for key, value in variables.items())
        preferred = ["part_ok", "vision_x", "vision_y", "vision_z", "vision_r", "vision_score", "vision_calibrated", "grip_ok", "count"]
        compact = ", ".join(f"{key}={variables[key]}" for key in preferred if key in variables)
        self.variables.setText(f"Variables: {compact or '-'}")
        self.variables.setToolTip(full)

    def _on_step_finished(self) -> None:
        self.refresh_table()
        self.refresh_preview()

    def refresh_preview(self) -> None:
        snapshot = self.devices.simulation_snapshot()
        if snapshot is not None:
            self.preview_canvas.set_state(
                snapshot.pose,
                snapshot.path,
                snapshot.gripper_closed,
                snapshot.backend,
                snapshot.physics_objects,
                snapshot.render_width,
                snapshot.render_height,
                snapshot.render_rgba,
            )
        else:
            self.preview_canvas.set_pose(self.devices.robot_pose())

    def refresh_analysis(self) -> None:
        return

    def _append_packet_log(self, packet: object) -> None:
        self.tcp_monitor.append_packet(packet)

    def _elapsed_seconds_text(self, elapsed_ms: float) -> str:
        return f"{elapsed_ms / 1000.0:.2f}"

    def _step_setting_text(self, step: SequenceStep) -> str:
        if step.device == "Vision":
            vision = step.vision_profile or self._default_vision_profile()
            calibration = step.calibration_profile or self._default_calibration_profile()
            return f"{step.target or 'D405_LOCATE'} | V:{vision} | C:{calibration}"
        return step.target

    def _wait_target_to_seconds(self, target: str) -> float:
        value = float(target or 0.0)
        if value > 60.0 and value <= 5000.0:
            return value / 1000.0
        return value
