from __future__ import annotations

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from core.device_manager import DeviceManager
from core.models import Pose, Position
from ui.dialogs.simulation_detail_dialog import SimulationDetailDialog
from ui.widgets.cards import labeled_row
from ui.widgets.robot_canvas import RobotCanvas


class PositionDialog(QDialog):
    def __init__(self, position: Position, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Position")
        layout = QFormLayout(self)
        self.name_edit = QLineEdit(position.name)
        self.motion = QComboBox()
        self.motion.addItems(["MoveJ", "MoveL"])
        self.motion.setCurrentText(position.motion_type)
        self.inputs: dict[str, QDoubleSpinBox] = {}
        for key, value in [("X", position.pose.x), ("Y", position.pose.y), ("Z", position.pose.z), ("R", position.pose.r)]:
            spin = QDoubleSpinBox()
            spin.setRange(-999.0, 999.0)
            spin.setDecimals(3)
            spin.setValue(value)
            self.inputs[key] = spin
            layout.addRow(key, spin)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setValue(position.speed)
        self.accel = QDoubleSpinBox()
        self.accel.setRange(1, 100)
        self.accel.setValue(position.acceleration)
        layout.insertRow(0, "Name", self.name_edit)
        layout.addRow("Motion", self.motion)
        layout.addRow("Speed", self.speed)
        layout.addRow("Accel", self.accel)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def apply_to(self, position: Position) -> None:
        position.name = self.name_edit.text().strip().upper() or position.name
        position.motion_type = self.motion.currentText()
        position.pose = Pose(self.inputs["X"].value(), self.inputs["Y"].value(), self.inputs["Z"].value(), self.inputs["R"].value())
        position.speed = self.speed.value()
        position.acceleration = self.accel.value()


class RobotPage(QWidget):
    positions_changed = Signal()

    def __init__(self, devices: DeviceManager) -> None:
        super().__init__()
        self.devices = devices
        self.positions: list[Position] = []
        self.target_inputs: dict[str, QDoubleSpinBox] = {}
        self.current_labels: dict[str, QLabel] = {}
        self.joint_labels: dict[str, QLabel] = {}
        self.sim_warning_label = QLabel("")
        self._last_canvas_move = 0.0

        self.canvas = RobotCanvas()
        self.canvas.set_interactive_enabled(False)
        self.canvas.target_pose_changed.connect(self._move_from_canvas)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_view_panel())
        splitter.addWidget(self._build_control_tabs())
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([980, 470])
        outer.addWidget(splitter, 3)
        outer.addWidget(self._build_position_table(), 2)
        self.refresh()

    def set_positions(self, positions: list[Position]) -> None:
        self.positions = positions
        self.refresh_positions()

    def _build_view_panel(self) -> QWidget:
        panel = QGroupBox("Robot View")
        layout = QVBoxLayout(panel)
        toolbar = QHBoxLayout()
        for name in ["Home", "Top", "Front", "Right", "Fit View"]:
            button = QPushButton(name)
            button.clicked.connect(lambda checked=False, n=name: self._set_workspace_view(n))
            toolbar.addWidget(button)
        self.render_scale_combo = QComboBox()
        self.render_scale_combo.addItems(["75%", "100%", "125%", "150%", "Fill"])
        self.render_scale_combo.setCurrentText("75%")
        self.render_scale_combo.setMinimumWidth(92)
        self.render_scale_combo.currentTextChanged.connect(self._set_render_scale)
        self.canvas.set_render_scale(0.75)
        toolbar.addWidget(QLabel("View"))
        toolbar.addWidget(self.render_scale_combo)
        detail = QPushButton("Detail")
        detail.setProperty("class", "primary")
        detail.clicked.connect(self._open_simulation_detail)
        toolbar.addWidget(detail)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        layout.addWidget(self.canvas, 1)
        self.sim_warning_label.setWordWrap(True)
        self.sim_warning_label.setProperty("class", "warn")
        layout.addWidget(self.sim_warning_label)
        return panel

    def _build_control_tabs(self) -> QWidget:
        self.control_tabs = QTabWidget()
        self.control_tabs.setMinimumWidth(430)
        self.control_tabs.addTab(self._scroll_page(self._build_move_tab()), "Move")
        self.control_tabs.addTab(self._scroll_page(self._build_jog_tab()), "Jog")
        self.control_tabs.addTab(self._scroll_page(self._build_robot_tab()), "Robot")
        return self.control_tabs

    def _scroll_page(self, page: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(page)
        return scroll

    def _build_move_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        current = QGroupBox("Current Pose")
        current_layout = QVBoxLayout(current)
        for label in ["X", "Y", "Z", "R"]:
            value = QLabel("-")
            self.current_labels[label] = value
            current_layout.addWidget(labeled_row(label, value))
        target = QGroupBox("Target Position")
        target_layout = QVBoxLayout(target)
        for name, value, suffix in [("X", 350.0, " mm"), ("Y", 100.0, " mm"), ("Z", 180.0, " mm"), ("R", 90.0, " deg")]:
            spin = QDoubleSpinBox()
            spin.setRange(-999.0, 999.0)
            spin.setDecimals(3)
            spin.setValue(value)
            spin.setSuffix(suffix)
            self.target_inputs[name] = spin
            target_layout.addWidget(labeled_row(name, spin))
        self.speed = QDoubleSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setValue(20)
        self.speed.setSuffix(" %")
        self.accel = QDoubleSpinBox()
        self.accel.setRange(1, 100)
        self.accel.setValue(50)
        self.accel.setSuffix(" %")
        target_layout.addWidget(labeled_row("Speed", self.speed))
        target_layout.addWidget(labeled_row("Accel", self.accel))
        buttons = QGridLayout()
        buttons.setHorizontalSpacing(8)
        read_current = QPushButton("Read Pos")
        read_current.setMinimumHeight(42)
        read_current.clicked.connect(self._read_current_to_target)
        buttons.addWidget(read_current, 0, 0)
        for col, (label, motion) in enumerate([("MoveJ", "MoveJ"), ("MoveL", "MoveL")], start=1):
            button = QPushButton(label)
            button.setProperty("class", "primary")
            button.setMinimumHeight(42)
            button.clicked.connect(lambda checked=False, m=motion: self._move(m))
            buttons.addWidget(button, 0, col)
        for col in range(3):
            buttons.setColumnStretch(col, 1)
        target_layout.addLayout(buttons)
        layout.addWidget(current)
        layout.addWidget(target)
        layout.addStretch()
        return page

    def _build_jog_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        jog = QGroupBox("Step Jog")
        jog_layout = QGridLayout(jog)
        jog_layout.setHorizontalSpacing(10)
        jog_layout.setVerticalSpacing(10)
        jog_layout.setColumnStretch(0, 1)
        jog_layout.setColumnStretch(1, 1)
        self.step = QComboBox()
        self.step.addItems(["0.1", "1", "5", "10"])
        self.step.setCurrentText("10")
        self.step.setMinimumContentsLength(16)
        self.step.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.step.setMinimumHeight(44)
        self.step.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        jog_layout.addWidget(labeled_row("Step", self.step), 0, 0, 1, 2)
        for row, axis in enumerate(["X", "Y", "Z", "R"], start=1):
            minus = QPushButton(f"{axis}-")
            plus = QPushButton(f"{axis}+")
            minus.setMinimumHeight(48)
            plus.setMinimumHeight(48)
            minus.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            plus.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            minus.clicked.connect(lambda checked=False, a=axis: self._jog(a, -1))
            plus.clicked.connect(lambda checked=False, a=axis: self._jog(a, 1))
            jog_layout.addWidget(minus, row, 0)
            jog_layout.addWidget(plus, row, 1)
        layout.addWidget(jog)
        layout.addStretch()
        return page

    def _build_robot_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        joints = QGroupBox("Joint Status")
        joints_layout = QVBoxLayout(joints)
        for label in ["J1", "J2", "J3", "J4"]:
            value = QLabel("-")
            self.joint_labels[label] = value
            joints_layout.addWidget(labeled_row(label, value))
        controls = QGroupBox("Robot Control")
        controls_layout = QVBoxLayout(controls)
        enable = QPushButton("Enable")
        enable.setProperty("class", "success")
        enable.clicked.connect(self.devices.enable_robot)
        disable = QPushButton("Disable")
        disable.clicked.connect(self.devices.disable_robot)
        clear = QPushButton("Clear Error")
        clear.clicked.connect(self.devices.clear_error)
        stop = QPushButton("Emergency Stop")
        stop.setProperty("class", "danger")
        stop.clicked.connect(self.devices.stop_robot)
        for button in [enable, disable, clear, stop]:
            button.setMinimumHeight(42)
            controls_layout.addWidget(button)
        info = QGroupBox("Workspace Limit")
        info_layout = QVBoxLayout(info)
        text = QLabel("Reach radius: 80..440 mm\nZ limit: -5..400 mm\nSoftware validation runs before every simulated move.")
        text.setWordWrap(True)
        info_layout.addWidget(text)
        layout.addWidget(joints)
        layout.addWidget(controls)
        layout.addWidget(info)
        layout.addStretch()
        return page

    def _build_position_table(self) -> QWidget:
        box = QGroupBox("Position Library")
        layout = QVBoxLayout(box)
        actions = QHBoxLayout()
        add = QPushButton("Add Current")
        edit = QPushButton("Edit")
        update = QPushButton("Update Selected")
        delete = QPushButton("Delete")
        add.clicked.connect(self._add_current)
        edit.clicked.connect(self._edit_selected)
        update.clicked.connect(self._update_selected)
        delete.clicked.connect(self._delete_selected)
        actions.addWidget(add)
        actions.addWidget(edit)
        actions.addWidget(update)
        actions.addWidget(delete)
        actions.addStretch()
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["No.", "Name", "X", "Y", "Z", "R"])
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addLayout(actions)
        layout.addWidget(self.table)
        return box

    def _set_workspace_view(self, name: str) -> None:
        self.canvas.set_view(name)
        self.devices.set_simulation_view(name)
        self.refresh()

    def _set_render_scale(self, value: str) -> None:
        if value == "Fill":
            self.canvas.set_render_scale(1.0, fill=True)
            return
        scale = float(value.rstrip("%")) / 100.0
        self.canvas.set_render_scale(scale)

    def refresh(self) -> None:
        pose = self.devices.robot_pose()
        joints = self.devices.robot_joints()
        snapshot = self.devices.simulation_snapshot()
        if snapshot is not None:
            self._apply_snapshot(snapshot)
        else:
            self.canvas.set_pose(pose)
            self.sim_warning_label.setText("")
            for name, value in zip(["X", "Y", "Z", "R"], [pose.x, pose.y, pose.z, pose.r]):
                self.current_labels[name].setText(f"{value:.3f}")
            for name, value in zip(["J1", "J2", "J3", "J4"], [joints.j1, joints.j2, joints.j3, joints.j4]):
                self.joint_labels[name].setText(f"{value:.2f}")

    def _apply_snapshot(self, snapshot) -> None:
        self.canvas.set_state(
            snapshot.pose,
            snapshot.path,
            snapshot.gripper_closed,
            snapshot.backend,
            snapshot.physics_objects,
            snapshot.render_width,
            snapshot.render_height,
            snapshot.render_rgba,
        )
        self.sim_warning_label.setText(" | ".join(snapshot.warnings) if snapshot.warnings else "")
        for name, value in zip(["X", "Y", "Z", "R"], [snapshot.pose.x, snapshot.pose.y, snapshot.pose.z, snapshot.pose.r]):
            self.current_labels[name].setText(f"{value:.3f}")
        for name, value in zip(["J1", "J2", "J3", "J4"], [snapshot.joints.j1, snapshot.joints.j2, snapshot.joints.j3, snapshot.joints.j4]):
            self.joint_labels[name].setText(f"{value:.2f}")

    def _set_target_inputs(self, pose: Pose) -> None:
        for name, value in zip(["X", "Y", "Z", "R"], [pose.x, pose.y, pose.z, pose.r]):
            self.target_inputs[name].blockSignals(True)
            self.target_inputs[name].setValue(value)
            self.target_inputs[name].blockSignals(False)

    def _open_simulation_detail(self) -> None:
        dialog = SimulationDetailDialog(self)
        dialog.exec()
        self.refresh()

    def refresh_positions(self) -> None:
        self.table.setRowCount(0)
        for index, position in enumerate(self.positions, start=1):
            row = self.table.rowCount()
            self.table.insertRow(row)
            values = [index, position.name, position.pose.x, position.pose.y, position.pose.z, position.pose.r]
            for col, value in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(str(value if col < 2 else f"{value:.3f}")))

    def _target_pose(self) -> Pose:
        return Pose(
            self.target_inputs["X"].value(),
            self.target_inputs["Y"].value(),
            self.target_inputs["Z"].value(),
            self.target_inputs["R"].value(),
        )

    def _move(self, motion: str) -> None:
        self.devices.move_robot(motion, self._target_pose(), self.speed.value(), self.accel.value())

    def _move_from_canvas(self, pose: Pose) -> None:
        self._set_target_inputs(pose)
        now = time.perf_counter()
        if now - self._last_canvas_move < 0.06:
            return
        self._last_canvas_move = now
        self.devices.move_robot("MoveL", pose, self.speed.value(), self.accel.value())

    def _jog(self, axis: str, direction: int) -> None:
        result = self.devices.jog_robot(axis, direction, float(self.step.currentText()))
        if result.success:
            self._set_target_inputs(self.devices.robot_pose())
        self.refresh()

    def _read_current_to_target(self) -> None:
        result = self.devices.refresh_robot_state()
        if result.success:
            self._set_target_inputs(self.devices.robot_pose())
        self.refresh()

    def _add_current(self) -> None:
        self.devices.refresh_robot_state()
        name, ok = QInputDialog.getText(self, "Add Position", "Position name:")
        if not ok or not name.strip():
            return
        self.positions.append(Position(name.strip().upper(), self.devices.robot_pose()))
        self.positions_changed.emit()
        self.refresh_positions()

    def _update_selected(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.positions):
            return
        self.devices.refresh_robot_state()
        self.positions[row].pose = self.devices.robot_pose()
        self.positions_changed.emit()
        self.refresh_positions()

    def _edit_selected(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.positions):
            return
        dialog = PositionDialog(self.positions[row], self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            dialog.apply_to(self.positions[row])
            self.positions_changed.emit()
            self.refresh_positions()

    def _delete_selected(self) -> None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self.positions):
            return
        del self.positions[row]
        self.positions_changed.emit()
        self.refresh_positions()

