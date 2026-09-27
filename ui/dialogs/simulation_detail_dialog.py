from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
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
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from core.models import Pose, Position
from simulation.robot_model import SimulationObject, ToolProfile
from ui.widgets.robot_canvas import RobotCanvas


class SimulationDetailDialog(QDialog):
    def __init__(self, page: "RobotPage") -> None:
        super().__init__(page)
        self.page = page
        self.devices = page.devices
        self.setWindowTitle("Simulation Detail")
        self.resize(1500, 860)
        self.setMinimumSize(1240, 720)
        self.setObjectName("simulationDetail")
        self.setStyleSheet(
            """
            QDialog#simulationDetail {
                background: #0b1220;
                color: #e5edf7;
            }
            QDialog#simulationDetail QWidget {
                background: #0b1220;
                color: #e5edf7;
            }
            QDialog#simulationDetail QTabWidget::pane {
                background: #0f172a;
                border: 1px solid #27364f;
                border-radius: 6px;
            }
            QDialog#simulationDetail QTabBar::tab {
                background: #172033;
                color: #cbd5e1;
                border: 1px solid #33445f;
                padding: 9px 16px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QDialog#simulationDetail QTabBar::tab:selected {
                background: #2563eb;
                color: #ffffff;
                border-color: #2563eb;
            }
            QDialog#simulationDetail QGroupBox {
                background: #0f172a;
                border: 1px solid #27364f;
                border-radius: 8px;
                margin-top: 10px;
                padding: 12px 10px 10px 10px;
                font-weight: 600;
            }
            QDialog#simulationDetail QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 6px;
                color: #93c5fd;
                background: #0b1220;
            }
            QDialog#simulationDetail QPushButton {
                background: #1f2a3d;
                color: #e5edf7;
                border: 1px solid #33445f;
                border-radius: 5px;
                padding: 8px 12px;
                min-height: 24px;
            }
            QDialog#simulationDetail QPushButton:hover {
                background: #26354d;
            }
            QDialog#simulationDetail QPushButton[class="primary"] {
                background: #2563eb;
                color: #ffffff;
                border-color: #2563eb;
            }
            QDialog#simulationDetail QLineEdit,
            QDialog#simulationDetail QDoubleSpinBox,
            QDialog#simulationDetail QComboBox {
                background: #111827;
                color: #f8fafc;
                border: 1px solid #33445f;
                border-radius: 5px;
                padding: 6px;
                min-height: 24px;
                selection-background-color: #2563eb;
                selection-color: #ffffff;
            }
            QDialog#simulationDetail QComboBox::drop-down {
                width: 24px;
                border-left: 1px solid #33445f;
            }
            QDialog#simulationDetail QTableWidget {
                background: #111827;
                color: #e5edf7;
                border: 1px solid #27364f;
                gridline-color: #243149;
                selection-background-color: #1d4ed8;
            }
            QDialog#simulationDetail QHeaderView::section {
                background: #172033;
                color: #e5edf7;
                border: none;
                border-right: 1px solid #27364f;
                border-bottom: 1px solid #27364f;
                padding: 6px;
            }
            QDialog#simulationDetail QSplitter::handle {
                background: #334155;
            }
            QDialog#simulationDetail QLabel#toolStatus {
                background: #111827;
                color: #cbd5e1;
                border: 1px solid #33445f;
                border-radius: 6px;
                padding: 8px;
            }
            QDialog#simulationDetail QLabel#previewTitle,
            QDialog#simulationDetail QLabel#previewStatus {
                color: #cbd5e1;
                background: transparent;
            }
            """
        )
        self.objects: list[SimulationObject] = []
        self.object_inputs: dict[str, QDoubleSpinBox | QLineEdit | QComboBox] = {}
        self.pose_inputs: dict[str, QDoubleSpinBox] = {}
        self.tool_name = QComboBox()
        self.tool_tcp_z = QDoubleSpinBox()
        self.tool_open = QDoubleSpinBox()
        self.tool_closed = QDoubleSpinBox()
        self.tool_range = QDoubleSpinBox()
        self.warning_label = QLabel("-")
        self.warning_label.setWordWrap(True)
        self._syncing_detail = False
        self._updating_table = False
        self.pose_apply_timer = QTimer(self)
        self.pose_apply_timer.setSingleShot(True)
        self.pose_apply_timer.setInterval(180)
        self.pose_apply_timer.timeout.connect(self._apply_pose_preview)
        self.tool_apply_timer = QTimer(self)
        self.tool_apply_timer.setSingleShot(True)
        self.tool_apply_timer.setInterval(220)
        self.tool_apply_timer.timeout.connect(self._apply_tool)
        self.object_apply_timer = QTimer(self)
        self.object_apply_timer.setSingleShot(True)
        self.object_apply_timer.setInterval(240)
        self.object_apply_timer.timeout.connect(self._apply_selected_object_live)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        preview_panel = self._preview_panel()
        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        tabs.setMinimumWidth(500)
        tabs.addTab(self._teaching_tab(), "Teach")
        tabs.addTab(self._tool_tab(), "Tool")
        tabs.addTab(self._objects_tab(), "Objects")
        tabs.addTab(self._checks_tab(), "Check")
        splitter.addWidget(preview_panel)
        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([940, 540])
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.setContentsMargins(6, 4, 6, 6)
        buttons.rejected.connect(self.reject)
        layout.addWidget(splitter, 1)
        layout.addWidget(buttons)
        self._load_from_snapshot()

    def _preview_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(660)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)
        title = QLabel("Live Simulation")
        title.setObjectName("previewTitle")
        title.setMinimumWidth(112)
        toolbar.addWidget(title)
        for name in ["Home", "Top", "Front", "Right", "Fit View"]:
            button = QPushButton(name)
            button.setMinimumHeight(30)
            button.setMinimumWidth(72)
            button.clicked.connect(lambda checked=False, n=name: self._set_preview_view(n))
            toolbar.addWidget(button)
        toolbar.addStretch()
        self.preview_status = QLabel("-")
        self.preview_status.setObjectName("previewStatus")
        self.preview_status.setMinimumWidth(180)
        self.preview_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        toolbar.addWidget(self.preview_status)
        self.preview_canvas = RobotCanvas()
        self.preview_canvas.setMinimumHeight(600)
        self.preview_canvas.set_render_scale(1.0, fill=True)
        self.preview_canvas.set_interactive_enabled(True)
        self.preview_canvas.target_pose_changed.connect(self._move_from_preview_canvas)
        layout.addLayout(toolbar)
        layout.addWidget(self.preview_canvas, 1)
        return panel

    def _teaching_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        pose_box = QGroupBox("Current Simulation Pose")
        pose_layout = QFormLayout(pose_box)
        self.teach_pose = QLabel("-")
        self.teach_name = QLineEdit("TEACH_1")
        add = QPushButton("Add Current To Position Library")
        read = QPushButton("Refresh Pose")
        add.clicked.connect(self._add_teach_position)
        read.clicked.connect(self._refresh_teach_pose)
        pose_layout.addRow("Pose", self.teach_pose)
        pose_layout.addRow("Name", self.teach_name)
        pose_layout.addRow(read, add)

        target_box = QGroupBox("Live Target Pose")
        target_layout = QFormLayout(target_box)
        for name, value, suffix in [("X", 300.0, " mm"), ("Y", 0.0, " mm"), ("Z", 250.0, " mm"), ("R", 0.0, " deg")]:
            spin = QDoubleSpinBox()
            spin.setRange(-999.0, 999.0)
            spin.setDecimals(3)
            spin.setValue(value)
            spin.setSuffix(suffix)
            spin.valueChanged.connect(self._preview_pose_from_inputs)
            self.pose_inputs[name] = spin
            target_layout.addRow(name, spin)
        self.detail_speed = QDoubleSpinBox()
        self.detail_speed.setRange(1.0, 100.0)
        self.detail_speed.setValue(self.page.speed.value())
        self.detail_speed.setSuffix(" %")
        self.detail_accel = QDoubleSpinBox()
        self.detail_accel.setRange(1.0, 100.0)
        self.detail_accel.setValue(self.page.accel.value())
        self.detail_accel.setSuffix(" %")
        target_layout.addRow("Speed", self.detail_speed)
        target_layout.addRow("Accel", self.detail_accel)

        jog_box = QGroupBox("Jog Teaching")
        jog_layout = QGridLayout(jog_box)
        self.detail_step = QComboBox()
        self.detail_step.addItems(["0.1", "1", "5", "10"])
        self.detail_step.setCurrentText(self.page.step.currentText())
        jog_layout.addWidget(QLabel("Step"), 0, 0)
        jog_layout.addWidget(self.detail_step, 0, 1)
        for row, axis in enumerate(["X", "Y", "Z", "R"], start=1):
            minus = QPushButton(f"{axis}-")
            plus = QPushButton(f"{axis}+")
            minus.clicked.connect(lambda checked=False, a=axis: self._detail_jog(a, -1))
            plus.clicked.connect(lambda checked=False, a=axis: self._detail_jog(a, 1))
            jog_layout.addWidget(minus, row, 0)
            jog_layout.addWidget(plus, row, 1)
        layout.addWidget(pose_box)
        layout.addWidget(target_box)
        layout.addWidget(jog_box)
        layout.addStretch()
        return page

    def _tool_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        form_box = QGroupBox("Tool Method")
        form = QFormLayout(form_box)
        form.setContentsMargins(10, 12, 10, 10)
        form.setSpacing(8)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.tool_form = form
        self.tool_name.addItems(["Soft Gripper", "Vacuum", "Custom"])
        for spin, value, suffix in [
            (self.tool_tcp_z, 65.0, " mm"),
            (self.tool_open, 42.0, " mm"),
            (self.tool_closed, 16.0, " mm"),
            (self.tool_range, 38.0, " mm"),
        ]:
            spin.setRange(0.0, 300.0)
            spin.setDecimals(2)
            spin.setValue(value)
            spin.setSuffix(suffix)
            spin.setMinimumWidth(220)
        apply = QPushButton("Apply Tool Profile")
        apply.setProperty("class", "primary")
        apply.clicked.connect(self._apply_tool)
        self.tool_status = QLabel("-")
        self.tool_status.setObjectName("toolStatus")
        self.tool_status.setWordWrap(True)
        self.tool_status.setMinimumHeight(44)
        self.tool_name.currentTextChanged.connect(self._on_tool_method_changed)
        for spin in [self.tool_tcp_z, self.tool_open, self.tool_closed, self.tool_range]:
            spin.valueChanged.connect(self._apply_tool_live)
        form.addRow("Profile", self.tool_name)
        form.addRow("TCP Offset Z", self.tool_tcp_z)
        form.addRow("Open Width", self.tool_open)
        form.addRow("Closed Width", self.tool_closed)
        form.addRow("Grip Range", self.tool_range)
        form.addRow("Preview", self.tool_status)
        layout.addWidget(form_box)
        layout.addWidget(apply)
        layout.addStretch()
        return page

    def _objects_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.object_table = QTableWidget(0, 8)
        self.object_table.setHorizontalHeaderLabels(["Name", "Shape", "X", "Y", "Z", "L/R", "W", "H"])
        header = self.object_table.horizontalHeader()
        for column, width in enumerate([110, 82, 70, 70, 70, 70, 70, 70]):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.object_table.setColumnWidth(column, width)
        self.object_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.object_table.currentCellChanged.connect(lambda *_: self._load_selected_object())

        editor = QGroupBox("Selected Object")
        form = QFormLayout(editor)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.object_inputs["name"] = QLineEdit()
        shape = QComboBox()
        shape.addItems(["Box", "Cylinder"])
        self.object_inputs["shape"] = shape
        for key, value in [("x", 300.0), ("y", 80.0), ("z", 25.0), ("size_x", 60.0), ("size_y", 60.0), ("size_z", 50.0), ("radius", 30.0)]:
            spin = QDoubleSpinBox()
            spin.setRange(-1000.0 if key in {"x", "y"} else 0.0, 1000.0)
            spin.setDecimals(2)
            spin.setValue(value)
            spin.setSuffix(" mm")
            self.object_inputs[key] = spin
        self.object_inputs["color"] = QLineEdit("#3b82f6")
        self.object_inputs["shape"].currentTextChanged.connect(self._update_selected_object_live)
        self.object_inputs["name"].editingFinished.connect(self._update_selected_object_live)
        self.object_inputs["color"].editingFinished.connect(self._update_selected_object_live)
        for key in ["x", "y", "z", "size_x", "size_y", "size_z", "radius"]:
            self.object_inputs[key].valueChanged.connect(self._update_selected_object_live)
        form.addRow("Name", self.object_inputs["name"])
        form.addRow("Shape", self.object_inputs["shape"])
        form.addRow("X", self.object_inputs["x"])
        form.addRow("Y", self.object_inputs["y"])
        form.addRow("Z", self.object_inputs["z"])
        form.addRow("Length / Radius", self.object_inputs["size_x"])
        form.addRow("Width", self.object_inputs["size_y"])
        form.addRow("Height", self.object_inputs["size_z"])
        form.addRow("Cylinder Radius", self.object_inputs["radius"])
        form.addRow("Color", self.object_inputs["color"])

        actions = QHBoxLayout()
        add_box = QPushButton("Add Box")
        add_cylinder = QPushButton("Add Cylinder")
        update = QPushButton("Update Selected")
        delete = QPushButton("Delete")
        reset = QPushButton("Reset Defaults")
        add_box.clicked.connect(lambda: self._add_object("Box"))
        add_cylinder.clicked.connect(lambda: self._add_object("Cylinder"))
        update.clicked.connect(self._update_selected_object)
        delete.clicked.connect(self._delete_selected_object)
        reset.clicked.connect(self._reset_objects)
        for button in [add_box, add_cylinder, update, delete, reset]:
            actions.addWidget(button)
        layout.addWidget(self.object_table, 1)
        layout.addWidget(editor)
        layout.addLayout(actions)
        return page

    def _checks_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        validate = QPushButton("Check Current Target")
        validate.clicked.connect(self._refresh_checks)
        layout.addWidget(validate)
        layout.addWidget(self.warning_label)
        layout.addStretch()
        return page

    def _load_from_snapshot(self) -> None:
        snapshot = self.devices.simulation_snapshot()
        if snapshot is None:
            return
        self._syncing_detail = True
        self.objects = [SimulationObject(**vars(item)) for item in snapshot.objects]
        self.tool_name.setCurrentText(snapshot.tool.name)
        self.tool_tcp_z.setValue(snapshot.tool.tcp_offset_z)
        self.tool_open.setValue(snapshot.tool.open_width)
        self.tool_closed.setValue(snapshot.tool.closed_width)
        self.tool_range.setValue(snapshot.tool.grip_range)
        self._set_pose_inputs(snapshot.pose)
        self._syncing_detail = False
        self._sync_tool_method_controls()
        self._refresh_teach_pose()
        self._refresh_object_table()
        self._refresh_preview()

    def _refresh_teach_pose(self) -> None:
        pose = self.devices.robot_pose()
        self.teach_pose.setText(f"X {pose.x:.3f}, Y {pose.y:.3f}, Z {pose.z:.3f}, R {pose.r:.3f}")
        self._set_pose_inputs(pose)

    def _set_pose_inputs(self, pose: Pose) -> None:
        if not self.pose_inputs:
            return
        for name, value in zip(["X", "Y", "Z", "R"], [pose.x, pose.y, pose.z, pose.r]):
            self.pose_inputs[name].blockSignals(True)
            self.pose_inputs[name].setValue(value)
            self.pose_inputs[name].blockSignals(False)

    def _add_teach_position(self) -> None:
        name = self.teach_name.text().strip().upper() or f"TEACH_{len(self.page.positions) + 1}"
        self.page.positions.append(Position(name, self.devices.robot_pose()))
        self.page.positions_changed.emit()
        self.page.refresh_positions()
        self.teach_name.setText(f"TEACH_{len(self.page.positions) + 1}")
        self._refresh_preview()

    def _apply_tool(self) -> None:
        if self._syncing_detail:
            return
        tool = ToolProfile(
            self.tool_name.currentText(),
            self.tool_tcp_z.value(),
            self.tool_open.value(),
            self.tool_closed.value(),
            self.tool_range.value(),
        )
        self.devices.configure_simulation_tool(tool)
        self._refresh_preview()

    def _apply_tool_live(self, *_args) -> None:
        if not self._syncing_detail:
            self.tool_apply_timer.start()

    def _on_tool_method_changed(self, *_args) -> None:
        self._sync_tool_method_controls()
        self._apply_tool_live()

    def _sync_tool_method_controls(self) -> None:
        is_vacuum = self.tool_name.currentText() == "Vacuum"
        self.tool_open.setVisible(not is_vacuum)
        self.tool_closed.setVisible(not is_vacuum)
        open_label = self.tool_form.labelForField(self.tool_open) if hasattr(self, "tool_form") else None
        closed_label = self.tool_form.labelForField(self.tool_closed) if hasattr(self, "tool_form") else None
        range_label = self.tool_form.labelForField(self.tool_range) if hasattr(self, "tool_form") else None
        if open_label is not None:
            open_label.setVisible(not is_vacuum)
        if closed_label is not None:
            closed_label.setVisible(not is_vacuum)
        if range_label is not None:
            range_label.setText("Vacuum Range" if is_vacuum else "Grip Range")
        if is_vacuum:
            for spin, value in [(self.tool_tcp_z, 80.0), (self.tool_open, 0.0), (self.tool_closed, 0.0), (self.tool_range, 55.0)]:
                spin.blockSignals(True)
                spin.setValue(value)
                spin.blockSignals(False)
        self.tool_status.setText(
            "Vacuum selected: teal suction pad is rendered under TCP. Finger width settings are hidden."
            if is_vacuum
            else "Soft Gripper selected: two fingers are rendered under TCP and width settings are active."
        )

    def _add_object(self, shape: str) -> None:
        index = len(self.objects) + 1
        if shape == "Cylinder":
            self.objects.append(SimulationObject(f"CYL_{index}", "Cylinder", 240.0, -80.0, 20.0, 40.0, 40.0, 40.0, 24.0, "#22c55e"))
        else:
            self.objects.append(SimulationObject(f"BOX_{index}", "Box", 320.0, 90.0, 25.0, 60.0, 60.0, 50.0, 30.0, "#3b82f6"))
        self._apply_objects()
        self._refresh_object_table()
        self.object_table.selectRow(len(self.objects) - 1)

    def _load_selected_object(self) -> None:
        if self._updating_table:
            return
        row = self.object_table.currentRow()
        if row < 0 or row >= len(self.objects):
            return
        item = self.objects[row]
        self._syncing_detail = True
        self.object_inputs["name"].setText(item.name)
        self.object_inputs["shape"].setCurrentText(item.shape)
        for key in ["x", "y", "z", "size_x", "size_y", "size_z", "radius"]:
            self.object_inputs[key].setValue(getattr(item, key))
        self.object_inputs["color"].setText(item.color)
        self._syncing_detail = False

    def _update_selected_object(self) -> None:
        self._apply_selected_object(refresh_table=True)

    def _apply_selected_object_live(self) -> None:
        self._apply_selected_object(refresh_table=False)

    def _apply_selected_object(self, refresh_table: bool) -> None:
        if self._syncing_detail:
            return
        row = self.object_table.currentRow()
        if row < 0 or row >= len(self.objects):
            return
        item = self.objects[row]
        item.name = self.object_inputs["name"].text().strip().upper() or item.name
        item.shape = self.object_inputs["shape"].currentText()
        for key in ["x", "y", "z", "size_x", "size_y", "size_z", "radius"]:
            setattr(item, key, self.object_inputs[key].value())
        item.color = self.object_inputs["color"].text().strip() or item.color
        item.attached = False
        self._apply_objects()
        if refresh_table:
            self._refresh_object_table()
            self.object_table.selectRow(row)

    def _update_selected_object_live(self, *_args) -> None:
        if not self._syncing_detail:
            self.object_apply_timer.start()

    def _delete_selected_object(self) -> None:
        row = self.object_table.currentRow()
        if row < 0 or row >= len(self.objects):
            return
        del self.objects[row]
        self._apply_objects()
        self._refresh_object_table()

    def _reset_objects(self) -> None:
        self.devices.reset_simulation_objects()
        self._load_from_snapshot()
        self.page.refresh()
        self._refresh_preview()

    def _apply_objects(self) -> None:
        self.devices.set_simulation_objects([SimulationObject(**vars(item)) for item in self.objects])
        self._refresh_preview()

    def _refresh_object_table(self) -> None:
        self._updating_table = True
        self.object_table.setRowCount(0)
        for item in self.objects:
            row = self.object_table.rowCount()
            self.object_table.insertRow(row)
            values = [
                item.name,
                item.shape,
                f"{item.x:.1f}",
                f"{item.y:.1f}",
                f"{item.z:.1f}",
                f"{item.radius:.1f}" if item.shape == "Cylinder" else f"{item.size_x:.1f}",
                f"{item.size_y:.1f}",
                f"{item.size_z:.1f}",
            ]
            for col, value in enumerate(values):
                table_item = QTableWidgetItem(value)
                table_item.setFlags(table_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.object_table.setItem(row, col, table_item)
        self._updating_table = False

    def _refresh_checks(self) -> None:
        warnings = self.devices.simulation_warnings_for_pose(self.page._target_pose())
        snapshot = self.devices.simulation_snapshot(render=False)
        if snapshot is not None:
            warnings = list(dict.fromkeys([*snapshot.warnings, *warnings]))
        self.warning_label.setText("\n".join(warnings) if warnings else "No simulation warnings for the current target.")
        if hasattr(self, "preview_status"):
            self.preview_status.setText(" | ".join(warnings) if warnings else "Ready")

    def _refresh_preview(self) -> None:
        if not hasattr(self, "preview_canvas"):
            return
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
            self._syncing_detail = True
            self._refresh_teach_pose()
            self._syncing_detail = False
            warnings = list(dict.fromkeys([*snapshot.warnings, *self.devices.simulation_warnings_for_pose(self.page._target_pose())]))
            self.warning_label.setText("\n".join(warnings) if warnings else "No simulation warnings for the current target.")
            self.preview_status.setText(" | ".join(warnings) if warnings else "Ready")
            self.page._apply_snapshot(snapshot)

    def _set_preview_view(self, name: str) -> None:
        self.preview_canvas.set_view(name)
        self.devices.set_simulation_view(name)
        self._refresh_preview()

    def _detail_target_pose(self) -> Pose:
        return Pose(
            self.pose_inputs["X"].value(),
            self.pose_inputs["Y"].value(),
            self.pose_inputs["Z"].value(),
            self.pose_inputs["R"].value(),
        )

    def _preview_pose_from_inputs(self, *_args) -> None:
        if self._syncing_detail or not self.pose_inputs:
            return
        self.pose_apply_timer.start()

    def _apply_pose_preview(self) -> None:
        self.devices.move_robot("MoveL", self._detail_target_pose(), self.detail_speed.value(), self.detail_accel.value())
        self._refresh_preview()

    def _move_from_preview_canvas(self, pose: Pose) -> None:
        self._set_pose_inputs(pose)
        self.devices.move_robot("MoveL", pose, self.detail_speed.value(), self.detail_accel.value())
        self._refresh_preview()

    def _detail_jog(self, axis: str, direction: int) -> None:
        result = self.devices.jog_robot(axis, direction, float(self.detail_step.currentText()))
        if result.success:
            self._set_pose_inputs(self.devices.robot_pose())
        self._refresh_preview()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.pose_apply_timer.stop()
        self.tool_apply_timer.stop()
        self.object_apply_timer.stop()
        super().closeEvent(event)

