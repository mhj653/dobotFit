from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QTextBrowser,
    QSizePolicy,
    QSlider,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from core.calibration_solver import calibration_errors, has_spatial_variation, robot_xyz, solve_camera_to_robot_matrix
from core.device_manager import DeviceManager
from core.vision_profiles import (
    DEFAULT_CALIBRATION_PROFILE,
    DEFAULT_VISION_PROFILE,
    default_calibration_store,
    default_vision_store,
    find_profile,
    normalize_calibration_store,
    normalize_vision_store,
    profile_names,
    replace_profile,
)
from drivers.camera.base import CalibrationConfig, CheckerboardConfig, DetectionConfig, detection_to_variables
from ui.widgets.image_view import ImageView
from ui.pages.vision_manuals import calibration_manual_html


DETECTION_ALGORITHMS = {
    "Blob": {
        "parameters": (
            "threshold",
            "polarity",
            "min_area",
            "max_area",
            "blur_kernel",
            "open_iterations",
            "close_iterations",
            "min_circularity",
        ),
    },
}


class VisionPage(QWidget):
    profiles_changed = Signal()

    def __init__(self, devices: DeviceManager) -> None:
        super().__init__()
        self.devices = devices
        self.vision_store = default_vision_store()
        self.calibration_store = default_calibration_store()
        self._syncing_profiles = False
        self.camera_to_robot_matrix = CalibrationConfig.identity().camera_to_robot[:]
        self.last_detection: dict[str, float | bool | str] = {}
        self.current_calibration_robot: tuple[float, float, float, float] | None = None
        self.current_calibration_robot_source = "Manual"
        self.calibration_manual_dialog: QDialog | None = None
        self.saved_robot_poses: list[tuple[float, float, float, float, str]] = []
        self.calibration_pairs: list[tuple[tuple[float, float, float], tuple[float, float, float, float]]] = []
        self.calibration_pair_sources: list[str] = []
        self.live_timer = QTimer(self)
        self.live_timer.setInterval(800)
        self.live_timer.timeout.connect(self._live_tick)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        header = QHBoxLayout()
        header.setSpacing(6)
        self.connect_toggle = QPushButton("D405 OFF")
        self.connect_toggle.setCheckable(True)
        self.connect_toggle.clicked.connect(self._toggle_realsense)
        self.capture_button = QPushButton("Capture")
        self.live_toggle = QPushButton("Live")
        self.live_toggle.setCheckable(True)
        self.auto_detect_check = QCheckBox("Auto Detect")
        self.capture_button.clicked.connect(lambda _checked=False: self._capture())
        self.live_toggle.clicked.connect(self._toggle_live)
        for button in [self.connect_toggle, self.live_toggle, self.capture_button]:
            button.setMinimumWidth(96)
            header.addWidget(button)
        header.addWidget(self.auto_detect_check)
        self.status = QLabel("RealSense D405: not connected")
        self.status.setWordWrap(False)
        self.status.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        header.addWidget(self.status, 1)
        layout.addLayout(header)

        tabs = QTabWidget()
        tabs.addTab(self._live_setup_tab(), "Live Vision")
        tabs.addTab(self._calibration_tab(), "Calibration")
        layout.addWidget(tabs, 1)
        self._refresh_project_profile_combos()
        self._sync_detection_controls()
        self._update_connection_ui()

    def _live_setup_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        live_panel = QWidget()
        live_layout = QVBoxLayout(live_panel)
        live_layout.setContentsMargins(0, 0, 0, 0)
        live_layout.setSpacing(6)
        image_grid = QGridLayout()
        image_grid.setContentsMargins(0, 0, 0, 0)
        image_grid.setSpacing(6)
        self.rgb_view = ImageView("Original Image")
        self.depth_view = ImageView("Depth Image")
        self.preprocessed_view = ImageView("Binary / Threshold")
        self.processed_view = ImageView("Result")
        for index, view in enumerate([self.rgb_view, self.depth_view, self.preprocessed_view, self.processed_view]):
            view.setMinimumHeight(170)
            view.roi_changed.connect(self._set_roi_from_view)
            image_grid.addWidget(view, index // 2, index % 2)
        image_grid.setRowStretch(0, 1)
        image_grid.setRowStretch(1, 1)
        image_grid.setColumnStretch(0, 1)
        image_grid.setColumnStretch(1, 1)
        live_layout.addLayout(image_grid, 1)

        setup_panel = QWidget()
        setup_panel.setMinimumWidth(430)
        setup_layout = QVBoxLayout(setup_panel)
        setup_layout.setContentsMargins(8, 0, 0, 0)
        setup_layout.setSpacing(6)

        test_box = QGroupBox("Vision Controls")
        test_box.setObjectName("visionPanel")
        test_layout = QGridLayout(test_box)
        test_layout.setContentsMargins(8, 12, 8, 8)
        test_layout.setHorizontalSpacing(6)
        test_layout.setVerticalSpacing(6)
        test_capture = QPushButton("Capture")
        self.set_roi_toggle = QPushButton("Set ROI")
        self.set_roi_toggle.setCheckable(True)
        test_capture.clicked.connect(lambda _checked=False: self._capture())
        self.set_roi_toggle.toggled.connect(self._set_roi_edit_mode)
        save_settings = QPushButton("Save")
        save_settings.setToolTip("Save Settings")
        load_settings = QPushButton("Load")
        load_settings.setToolTip("Load Settings")
        save_settings.clicked.connect(self._save_vision_settings)
        load_settings.clicked.connect(self._load_vision_settings)
        self.test_summary = QLabel("Result: -")
        self.test_summary.setWordWrap(True)
        self.test_summary.setMaximumHeight(42)
        self.vision_profile_combo = QComboBox()
        self.vision_profile_combo.setEditable(True)
        self.vision_profile_combo.currentTextChanged.connect(self._on_vision_profile_selected)
        load_vision_profile = QPushButton("Load Profile")
        save_vision_profile = QPushButton("Save Profile")
        load_vision_profile.clicked.connect(self._request_load_selected_vision_profile)
        save_vision_profile.clicked.connect(self._save_current_vision_profile)
        for index, button in enumerate([test_capture, self.set_roi_toggle, save_settings, load_settings]):
            button.setProperty("vision", "true")
            button.setMinimumWidth(0)
            test_layout.addWidget(button, 0, index)
        test_layout.addWidget(self.test_summary, 1, 0, 1, 4)
        test_layout.addWidget(QLabel("Profile"), 2, 0)
        test_layout.addWidget(self.vision_profile_combo, 2, 1)
        test_layout.addWidget(load_vision_profile, 2, 2)
        test_layout.addWidget(save_vision_profile, 2, 3)

        detection = QGroupBox("Detection Method")
        detection.setObjectName("visionPanel")
        form = QFormLayout(detection)
        form.setContentsMargins(8, 12, 8, 8)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(5)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.method_combo = QComboBox()
        self.method_combo.setProperty("vision", "true")
        self.method_combo.addItems(list(DETECTION_ALGORITHMS.keys()))
        self.method_combo.currentTextChanged.connect(self._on_detection_setup_changed)
        self.method_label = QLabel(self.method_combo.currentText())
        self.method_label.setFixedWidth(52)
        self.blob_polarity = QComboBox()
        self.blob_polarity.setProperty("vision", "true")
        self.blob_polarity.setFixedWidth(86)
        self.blob_polarity.addItems(["Auto", "Bright", "Dark"])
        self.polarity_label = QLabel("Polarity")
        test_detect = QPushButton("Locate")
        test_detect.setToolTip("Test Locate")
        test_detect.setProperty("vision", "true")
        test_detect.setProperty("class", "primary")
        test_detect.clicked.connect(self._detect)
        test_detect.setFixedWidth(96)
        method_row = QWidget()
        method_layout = QHBoxLayout(method_row)
        method_layout.setContentsMargins(0, 0, 0, 0)
        method_layout.setSpacing(6)
        if len(DETECTION_ALGORITHMS) > 1:
            method_layout.addWidget(self.method_combo, 1)
        else:
            self.method_combo.setVisible(False)
            method_layout.addWidget(self.method_label, 1)
        method_layout.addWidget(self.polarity_label)
        method_layout.addWidget(self.blob_polarity)
        method_layout.addWidget(test_detect)
        self.threshold_row, self.threshold, self.threshold_slider = self._slider_spin_control(0.0, 255.0, 140.0, 1.0, 0)
        self.min_area_row, self.min_area, self.min_area_slider = self._slider_spin_control(1.0, 50000.0, 80.0, 10.0, 0)
        self.max_area_row, self.max_area, self.max_area_slider = self._slider_spin_control(1.0, 307200.0, 20000.0, 100.0, 0)
        self.blur_kernel_row, self.blur_kernel, self.blur_kernel_slider = self._slider_spin_control(0.0, 15.0, 5.0, 2.0, 0)
        self.open_iterations_row, self.open_iterations, self.open_iterations_slider = self._slider_spin_control(0.0, 5.0, 0.0, 1.0, 0)
        self.close_iterations_row, self.close_iterations, self.close_iterations_slider = self._slider_spin_control(0.0, 5.0, 0.0, 1.0, 0)
        self.min_circularity_row, self.min_circularity, self.min_circularity_slider = self._slider_spin_control(0.0, 1.0, 0.0, 0.01, 2, slider_scale=100)
        self.threshold_label = QLabel("Threshold")
        self.min_area_label = QLabel("Min Area")
        self.max_area_label = QLabel("Max Area")
        self.blur_kernel_label = QLabel("Blur")
        self.open_iterations_label = QLabel("Open")
        self.close_iterations_label = QLabel("Close")
        self.min_circularity_label = QLabel("Circularity")
        form.addRow("Method", method_row)
        form.addRow(self.threshold_label, self.threshold_row)
        form.addRow(self.min_area_label, self.min_area_row)
        form.addRow(self.max_area_label, self.max_area_row)
        form.addRow(self.blur_kernel_label, self.blur_kernel_row)
        form.addRow(self.open_iterations_label, self.open_iterations_row)
        form.addRow(self.close_iterations_label, self.close_iterations_row)
        form.addRow(self.min_circularity_label, self.min_circularity_row)
        self.detection_parameter_widgets = {
            "threshold": [self.threshold_label, self.threshold_row],
            "polarity": [self.polarity_label, self.blob_polarity],
            "min_area": [self.min_area_label, self.min_area_row],
            "max_area": [self.max_area_label, self.max_area_row],
            "blur_kernel": [self.blur_kernel_label, self.blur_kernel_row],
            "open_iterations": [self.open_iterations_label, self.open_iterations_row],
            "close_iterations": [self.close_iterations_label, self.close_iterations_row],
            "min_circularity": [self.min_circularity_label, self.min_circularity_row],
        }

        self.roi_x = self._spin(0.0, 100.0, 35.0, 1.0)
        self.roi_y = self._spin(0.0, 100.0, 30.0, 1.0)
        self.roi_w = self._spin(1.0, 100.0, 30.0, 1.0)
        self.roi_h = self._spin(1.0, 100.0, 40.0, 1.0)
        for widget in [self.roi_x, self.roi_y, self.roi_w, self.roi_h]:
            widget.setSuffix(" %")
            widget.valueChanged.connect(self._on_detection_setup_changed)

        self.threshold.valueChanged.connect(self._on_detection_setup_changed)
        self.min_area.valueChanged.connect(self._on_detection_setup_changed)
        self.max_area.valueChanged.connect(self._on_detection_setup_changed)
        self.blob_polarity.currentTextChanged.connect(self._on_detection_setup_changed)
        self.blur_kernel.valueChanged.connect(self._on_detection_setup_changed)
        self.open_iterations.valueChanged.connect(self._on_detection_setup_changed)
        self.close_iterations.valueChanged.connect(self._on_detection_setup_changed)
        self.min_circularity.valueChanged.connect(self._on_detection_setup_changed)
        setup_layout.addWidget(test_box)
        setup_layout.addWidget(detection)
        setup_layout.addWidget(self._result_box())
        setup_layout.addStretch(1)

        splitter.addWidget(live_panel)
        splitter.addWidget(setup_panel)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1160, 500])
        layout.addWidget(splitter, 1)
        return page

    def _calibration_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        workflow_box = QGroupBox("Calibration Workflow")
        workflow_grid = QGridLayout(workflow_box)
        workflow_grid.setContentsMargins(8, 10, 8, 8)
        workflow_grid.setHorizontalSpacing(6)
        workflow_grid.setVerticalSpacing(5)
        self.calibration_method_combo = QComboBox()
        self.calibration_method_combo.addItems(["Point Pair", "Checkerboard"])
        self.calibration_method_combo.currentTextChanged.connect(self._sync_calibration_method)
        self.calibration_status_label = QLabel("Need 3 or more point pairs")
        self.calibration_status_label.setProperty("class", "warn")
        self.calibration_pair_count_label = QLabel("Pairs: 0")
        self.calibration_quality_label = QLabel("Quality: -")
        self.calibration_camera_label = QLabel("Camera XYZ: -")
        self.calibration_robot_label = QLabel("Robot XYZ: -")
        for label in [
            self.calibration_status_label,
            self.calibration_pair_count_label,
            self.calibration_quality_label,
            self.calibration_camera_label,
            self.calibration_robot_label,
        ]:
            label.setWordWrap(True)
        self.detect_target_button = QPushButton("1 Detect Target")
        self.read_robot_button = QPushButton("2 GetPose")
        self.add_pair_button = QPushButton("3 Add Pair")
        solve = QPushButton("4 Solve && Apply")
        clear = QPushButton("Clear")
        self.calibration_manual_button = QPushButton("Manual")
        solve.setProperty("class", "primary")
        clear.setProperty("class", "danger")
        self.detect_target_button.clicked.connect(self._detect_calibration_target)
        self.read_robot_button.clicked.connect(self._read_calibration_robot_pose)
        self.add_pair_button.clicked.connect(self._add_calibration_pair)
        solve.clicked.connect(self._solve_calibration_matrix)
        clear.clicked.connect(self._clear_calibration_pairs)
        self.calibration_manual_button.clicked.connect(self._open_calibration_manual)
        self.calibration_profile_combo = QComboBox()
        self.calibration_profile_combo.setEditable(True)
        self.calibration_profile_combo.currentTextChanged.connect(self._on_calibration_profile_selected)
        load_calibration_profile = QPushButton("Load Profile")
        save_calibration_profile = QPushButton("Save Profile")
        load_calibration_profile.clicked.connect(self._request_load_selected_calibration_profile)
        save_calibration_profile.clicked.connect(self._save_current_calibration_profile)
        for button in [self.detect_target_button, self.read_robot_button, self.add_pair_button, solve, clear, self.calibration_manual_button]:
            button.setMinimumHeight(34)
            button.setFixedWidth(122)
        self.calibration_method_combo.setFixedWidth(140)
        workflow_grid.addWidget(QLabel("Method"), 0, 0)
        workflow_grid.addWidget(self.calibration_method_combo, 0, 1)
        workflow_grid.addWidget(self.calibration_status_label, 0, 2, 1, 3)
        workflow_grid.addWidget(self.calibration_pair_count_label, 0, 5)
        workflow_grid.addWidget(self.detect_target_button, 1, 0)
        workflow_grid.addWidget(self.read_robot_button, 1, 1)
        workflow_grid.addWidget(self.add_pair_button, 1, 2)
        workflow_grid.addWidget(solve, 1, 3)
        workflow_grid.addWidget(clear, 1, 4)
        workflow_grid.addWidget(self.calibration_manual_button, 1, 5)
        workflow_grid.addWidget(self.calibration_camera_label, 2, 0, 1, 3)
        workflow_grid.addWidget(self.calibration_robot_label, 2, 3, 1, 3)
        workflow_grid.addWidget(self.calibration_quality_label, 3, 0, 1, 6)
        workflow_grid.addWidget(QLabel("Profile"), 4, 0)
        workflow_grid.addWidget(self.calibration_profile_combo, 4, 1, 1, 3)
        workflow_grid.addWidget(load_calibration_profile, 4, 4)
        workflow_grid.addWidget(save_calibration_profile, 4, 5)
        workflow_grid.setColumnStretch(2, 1)
        workflow_grid.setColumnStretch(3, 1)
        layout.addWidget(workflow_box, 0)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        capture_panel = QWidget()
        capture_layout = QVBoxLayout(capture_panel)
        capture_layout.setContentsMargins(0, 0, 6, 0)
        capture_layout.setSpacing(6)

        self.checkerboard_box = QGroupBox("Checkerboard Settings")
        checker_grid = QGridLayout(self.checkerboard_box)
        checker_grid.setContentsMargins(8, 10, 8, 8)
        checker_grid.setHorizontalSpacing(6)
        checker_grid.setVerticalSpacing(4)
        self.checkerboard_cols = self._spin(2.0, 30.0, 7.0, 1.0)
        self.checkerboard_rows = self._spin(2.0, 30.0, 6.0, 1.0)
        self.checkerboard_square = self._spin(1.0, 200.0, 20.0, 1.0)
        self.checkerboard_cols.setDecimals(0)
        self.checkerboard_rows.setDecimals(0)
        self.checkerboard_square.setSuffix(" mm")
        checker_grid.addWidget(QLabel("Inner X"), 0, 0)
        checker_grid.addWidget(self.checkerboard_cols, 0, 1)
        checker_grid.addWidget(QLabel("Inner Y"), 0, 2)
        checker_grid.addWidget(self.checkerboard_rows, 0, 3)
        checker_grid.addWidget(QLabel("Square"), 0, 4)
        checker_grid.addWidget(self.checkerboard_square, 0, 5)
        self.checkerboard_box.setMaximumHeight(76)

        self.calibration_camera_box = QGroupBox("1 Detect Target Image")
        calibration_camera_layout = QVBoxLayout(self.calibration_camera_box)
        calibration_camera_layout.setContentsMargins(8, 10, 8, 8)
        calibration_camera_layout.setSpacing(6)
        calibration_images = QWidget()
        calibration_images_layout = QHBoxLayout(calibration_images)
        calibration_images_layout.setContentsMargins(0, 0, 0, 0)
        calibration_images_layout.setSpacing(6)
        self.calibration_view = ImageView("1 Detect Point / Result")
        self.calibration_pair_view = ImageView("3 Added Pair Snapshot")
        for view in [self.calibration_view, self.calibration_pair_view]:
            view.setMinimumHeight(180)
        calibration_images_layout.addWidget(self.calibration_view, 1)
        calibration_images_layout.addWidget(self.calibration_pair_view, 1)
        self.calibration_live_toggle = QPushButton("Live Preview")
        self.calibration_live_toggle.setCheckable(True)
        self.calibration_live_toggle.clicked.connect(self._toggle_calibration_live)
        calibration_camera_layout.addWidget(calibration_images, 1)
        calibration_camera_layout.addWidget(self.calibration_live_toggle)

        robot_pose_box = QGroupBox("Robot Pose Input")
        robot_pose_grid = QGridLayout(robot_pose_box)
        robot_pose_grid.setContentsMargins(8, 10, 8, 8)
        robot_pose_grid.setHorizontalSpacing(6)
        robot_pose_grid.setVerticalSpacing(5)
        self.cal_robot_x = self._spin(-999999.0, 999999.0, 0.0, 1.0)
        self.cal_robot_y = self._spin(-999999.0, 999999.0, 0.0, 1.0)
        self.cal_robot_z = self._spin(-999999.0, 999999.0, 0.0, 1.0)
        self.cal_robot_rz = self._spin(-999999.0, 999999.0, 0.0, 1.0)
        for spin in [self.cal_robot_x, self.cal_robot_y, self.cal_robot_z, self.cal_robot_rz]:
            spin.setDecimals(3)
            spin.setMinimumWidth(96)
            spin.valueChanged.connect(self._on_calibration_robot_input_changed)
        robot_pose_grid.addWidget(QLabel("X"), 0, 0)
        robot_pose_grid.addWidget(self.cal_robot_x, 0, 1)
        robot_pose_grid.addWidget(QLabel("Y"), 0, 2)
        robot_pose_grid.addWidget(self.cal_robot_y, 0, 3)
        robot_pose_grid.addWidget(QLabel("Z"), 0, 4)
        robot_pose_grid.addWidget(self.cal_robot_z, 0, 5)
        robot_pose_grid.addWidget(QLabel("RZ"), 0, 6)
        robot_pose_grid.addWidget(self.cal_robot_rz, 0, 7)
        get_pose = QPushButton("GetPose")
        save_pose = QPushButton("Save Pose")
        use_selected_pose = QPushButton("Use Selected")
        get_pose.clicked.connect(self._read_calibration_robot_pose)
        save_pose.clicked.connect(self._save_calibration_robot_pose)
        use_selected_pose.clicked.connect(self._use_selected_robot_pose)
        robot_pose_grid.addWidget(get_pose, 1, 0, 1, 2)
        robot_pose_grid.addWidget(save_pose, 1, 2, 1, 2)
        robot_pose_grid.addWidget(use_selected_pose, 1, 4, 1, 4)
        robot_pose_box.setMaximumHeight(126)

        capture_layout.addWidget(self.checkerboard_box, 0)
        capture_layout.addWidget(self.calibration_camera_box, 1)
        capture_layout.addWidget(robot_pose_box, 0)

        data_panel = QWidget()
        data_layout = QVBoxLayout(data_panel)
        data_layout.setContentsMargins(6, 0, 0, 0)
        data_layout.setSpacing(6)

        matrix_box = QGroupBox("Camera To Robot 4x4 Matrix")
        matrix_layout = QVBoxLayout(matrix_box)
        matrix_layout.setContentsMargins(8, 10, 8, 8)
        matrix_layout.setSpacing(6)
        self.matrix_summary = QLabel()
        self.matrix_summary.setWordWrap(True)
        matrix_settings = QPushButton("Matrix Settings")
        matrix_settings.clicked.connect(self._open_matrix_dialog)
        self.calibrated_check = QCheckBox("Use solved matrix for robot XYZ")
        apply = QPushButton("Apply Calibration")
        apply.setProperty("class", "primary")
        apply.clicked.connect(self._apply_calibration)
        matrix_actions = QHBoxLayout()
        matrix_actions.addWidget(self.calibrated_check, 1)
        matrix_actions.addWidget(matrix_settings)
        matrix_actions.addWidget(apply)
        note = QLabel("Camera XYZ is transformed to robot XYZ by the saved matrix. RZ is stored with samples for reference.")
        note.setWordWrap(True)
        matrix_layout.addWidget(self.matrix_summary)
        matrix_layout.addLayout(matrix_actions)
        matrix_layout.addWidget(note)

        table_tabs = QTabWidget()
        saved_pose_tab = QWidget()
        saved_pose_layout = QVBoxLayout(saved_pose_tab)
        saved_pose_layout.setContentsMargins(4, 4, 4, 4)
        self.robot_pose_table = QTableWidget(0, 6)
        self.robot_pose_table.setHorizontalHeaderLabels(["#", "X", "Y", "Z", "RZ", "Source"])
        pose_header = self.robot_pose_table.horizontalHeader()
        pose_header.setStretchLastSection(False)
        for column, width in enumerate([40, 82, 82, 82, 82, 96]):
            pose_header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.robot_pose_table.setColumnWidth(column, width)
        self.robot_pose_table.setAlternatingRowColors(True)
        self.robot_pose_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.robot_pose_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        pose_actions = QHBoxLayout()
        use_pose_from_table = QPushButton("Use Selected Pose")
        remove_pose = QPushButton("Remove Pose")
        use_pose_from_table.clicked.connect(self._use_selected_robot_pose)
        remove_pose.clicked.connect(self._delete_selected_robot_pose)
        pose_actions.addWidget(use_pose_from_table)
        pose_actions.addWidget(remove_pose)
        pose_actions.addStretch()
        saved_pose_layout.addWidget(self.robot_pose_table)
        saved_pose_layout.addLayout(pose_actions)

        pair_tab = QWidget()
        pair_layout = QVBoxLayout(pair_tab)
        pair_layout.setContentsMargins(4, 4, 4, 4)
        self.pair_table = QTableWidget(0, 10)
        self.pair_table.setHorizontalHeaderLabels(["#", "Type", "Cam X", "Cam Y", "Cam Z", "Robot X", "Robot Y", "Robot Z", "Robot RZ", "Error"])
        pair_header = self.pair_table.horizontalHeader()
        pair_header.setStretchLastSection(False)
        for column, width in enumerate([36, 92, 82, 82, 82, 82, 82, 82, 82, 70]):
            pair_header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.pair_table.setColumnWidth(column, width)
        self.pair_table.setAlternatingRowColors(True)
        self.pair_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.pair_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        pair_actions = QHBoxLayout()
        remove_pair = QPushButton("Remove Selected Pair")
        clear_pairs = QPushButton("Clear All Pairs")
        remove_pair.clicked.connect(self._delete_selected_calibration_pair)
        clear_pairs.clicked.connect(self._clear_calibration_pairs)
        pair_actions.addWidget(remove_pair)
        pair_actions.addWidget(clear_pairs)
        pair_actions.addStretch()
        pair_layout.addWidget(self.pair_table)
        pair_layout.addLayout(pair_actions)

        table_tabs.addTab(saved_pose_tab, "Saved Robot Poses")
        self.pair_tab_index = table_tabs.addTab(pair_tab, "Point Pairs")
        self.table_tabs = table_tabs
        data_layout.addWidget(matrix_box, 0)
        data_layout.addWidget(table_tabs, 1)

        splitter.addWidget(capture_panel)
        splitter.addWidget(data_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([850, 680])
        layout.addWidget(splitter, 1)
        self._refresh_matrix_summary()
        self._sync_calibration_method()
        self._update_calibration_status()
        return page

    def _result_box(self) -> QGroupBox:
        box = QGroupBox("Detection Result")
        box.setObjectName("visionPanel")
        layout = QGridLayout(box)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        self.result_rows: dict[str, QLabel] = {}
        keys = [
            ("part_ok", "OK"),
            ("vision_score", "Score"),
            ("vision_pixel_u", "Pixel U"),
            ("vision_pixel_v", "Pixel V"),
            ("vision_depth_mm", "Depth"),
            ("vision_x", "Robot X"),
            ("vision_y", "Robot Y"),
            ("vision_z", "Robot Z"),
            ("vision_camera_x_mm", "Cam X"),
            ("vision_camera_y_mm", "Cam Y"),
            ("vision_camera_z_mm", "Cam Z"),
            ("vision_calibrated", "Cal"),
            ("vision_message", "Message"),
        ]
        for index, (key, title) in enumerate(keys):
            row = index // 2
            col = (index % 2) * 2
            name = QLabel(title)
            name.setProperty("class", "muted")
            value = QLabel("-")
            value.setWordWrap(key == "vision_message")
            if key == "vision_message":
                value.setMaximumHeight(34)
            self.result_rows[key] = value
            layout.addWidget(name, row, col)
            layout.addWidget(value, row, col + 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)
        return box

    def _spin(self, minimum: float, maximum: float, value: float, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _slider_spin_control(
        self,
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        decimals: int,
        slider_scale: int = 1,
    ) -> tuple[QWidget, QDoubleSpinBox, QSlider]:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setMinimumWidth(80)
        slider.setRange(int(round(minimum * slider_scale)), int(round(maximum * slider_scale)))
        spin = self._spin(minimum, maximum, value, step)
        spin.setProperty("vision", "true")
        spin.setDecimals(decimals)
        spin.setFixedWidth(82)
        slider.setValue(int(round(value * slider_scale)))

        def slider_to_spin(raw_value: int) -> None:
            spin.blockSignals(True)
            spin.setValue(raw_value / slider_scale)
            spin.blockSignals(False)
            self._on_detection_setup_changed()

        def spin_to_slider(raw_value: float) -> None:
            slider.blockSignals(True)
            slider.setValue(int(round(raw_value * slider_scale)))
            slider.blockSignals(False)

        slider.valueChanged.connect(slider_to_spin)
        spin.valueChanged.connect(spin_to_slider)
        layout.addWidget(slider, 1)
        layout.addWidget(spin)
        return row, spin, slider

    def _toggle_realsense(self, checked: bool) -> None:
        if checked:
            result = self.devices.connect_realsense()
            self.status.setText(result.message)
            if result.success:
                self._capture()
            else:
                self.connect_toggle.setChecked(False)
        else:
            self.live_toggle.setChecked(False)
            self.live_timer.stop()
            disconnect = getattr(self.devices.camera3d, "disconnect", None)
            result = disconnect() if callable(disconnect) else None
            self.status.setText(result.message if result is not None else "RealSense D405 disconnected")
            self.rgb_view.set_frame(None)
            self.depth_view.set_frame(None)
            self.preprocessed_view.set_frame(None)
            self.processed_view.set_frame(None)
        self._update_connection_ui()

    def _capture(self, update_status: bool = True) -> bool:
        self._apply_detection_config()
        if not self._camera_connected():
            result = self.devices.connect_realsense()
            if not result.success:
                self.status.setText(result.message)
                self.connect_toggle.setChecked(False)
                self._update_connection_ui()
                return False
            self.connect_toggle.setChecked(True)
        result = self.devices.camera3d.capture()
        if update_status:
            self.status.setText(result.message)
        self._update_connection_ui()
        self._refresh_views()
        return result.success

    def _detect(self) -> None:
        self._apply_detection_config()
        self._apply_calibration()
        if not self._camera_connected():
            result = self.devices.connect_realsense()
            if not result.success:
                self.status.setText(result.message)
                self.connect_toggle.setChecked(False)
                self._update_connection_ui()
                return
            self.connect_toggle.setChecked(True)
        values = detection_to_variables(self.devices.camera2d.detect())
        self.last_detection = values
        self.status.setText(str(values.get("vision_message", "Vision detect complete")))
        self._update_test_summary(values)
        self._update_connection_ui()
        self._update_result_rows(values)
        self._refresh_views()

    def _apply_detection_config(self) -> None:
        config = DetectionConfig(
            method=self.method_combo.currentText(),
            roi_x_percent=self.roi_x.value(),
            roi_y_percent=self.roi_y.value(),
            roi_w_percent=self.roi_w.value(),
            roi_h_percent=self.roi_h.value(),
            threshold=self.threshold.value() / 255.0,
            min_area=self.min_area.value(),
            max_area=self.max_area.value(),
            blob_polarity=self.blob_polarity.currentText(),
            blur_kernel=int(self.blur_kernel.value()),
            open_iterations=int(self.open_iterations.value()),
            close_iterations=int(self.close_iterations.value()),
            min_circularity=self.min_circularity.value(),
        )
        configure = getattr(self.devices.camera2d, "configure_detection", None)
        if callable(configure):
            configure(config)
        preview = getattr(self.devices.camera2d, "refresh_detection_preview", None)
        if callable(preview):
            preview()
        self._refresh_views()

    def _apply_calibration(self) -> None:
        config = CalibrationConfig(list(self.camera_to_robot_matrix), self.calibrated_check.isChecked())
        configure = getattr(self.devices.camera2d, "configure_calibration", None)
        if callable(configure):
            configure(config)
        self._refresh_pair_table()
        self._update_calibration_status()

    def _open_matrix_dialog(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle("Camera To Robot 4x4 Matrix")
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        grid = QGridLayout()
        grid.setSpacing(6)
        inputs: list[QDoubleSpinBox] = []
        for row in range(4):
            for col in range(4):
                spin = self._spin(-999999.0, 999999.0, self.camera_to_robot_matrix[row * 4 + col], 1.0)
                spin.setDecimals(6)
                spin.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
                spin.setMinimumWidth(120)
                inputs.append(spin)
                grid.addWidget(spin, row, col)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addLayout(grid)
        layout.addWidget(buttons)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._set_camera_to_robot_matrix([spin.value() for spin in inputs])
            self._apply_calibration()
            self.status.setText("Camera-to-robot matrix saved and applied")

    def _set_camera_to_robot_matrix(self, matrix: list[float]) -> None:
        if len(matrix) != 16:
            raise ValueError("camera_to_robot must have 16 values")
        self.camera_to_robot_matrix = [float(value) for value in matrix]
        self._refresh_matrix_summary()

    def _refresh_matrix_summary(self) -> None:
        if not hasattr(self, "matrix_summary"):
            return
        rows = []
        matrix = self.camera_to_robot_matrix
        for index in range(0, 16, 4):
            rows.append("[" + ", ".join(f"{value:.3f}" for value in matrix[index : index + 4]) + "]")
        self.matrix_summary.setText("\n".join(rows))

    def _refresh_views(self) -> None:
        roi = (self.roi_x.value(), self.roi_y.value(), self.roi_w.value(), self.roi_h.value())
        marker = None
        try:
            if self.last_detection.get("part_ok"):
                marker = (float(self.last_detection["vision_pixel_u"]), float(self.last_detection["vision_pixel_v"]))
        except (KeyError, TypeError, ValueError):
            marker = None
        self.rgb_view.set_overlay(roi, marker)
        self.depth_view.set_overlay(roi, marker)
        self.preprocessed_view.set_overlay(roi, marker)
        self.processed_view.set_overlay(roi, marker)

        color_frame = getattr(self.devices.camera2d, "color_frame", lambda: None)()
        depth_frame = getattr(self.devices.camera3d, "depth_preview_frame", lambda: None)()
        preprocessed_frame = getattr(self.devices.camera2d, "preprocessed_preview_frame", lambda: None)()
        processed_frame = getattr(self.devices.camera2d, "result_preview_frame", lambda: None)()
        self.rgb_view.set_frame(color_frame)
        self.depth_view.set_frame(depth_frame)
        self.preprocessed_view.set_frame(preprocessed_frame)
        self.processed_view.set_frame(processed_frame)
        if hasattr(self, "calibration_view"):
            self.calibration_view.set_overlay(None, marker)
            self.calibration_view.set_frame(processed_frame if processed_frame is not None else color_frame)

    def _update_result_rows(self, values: dict[str, float | bool | str]) -> None:
        for key, label in self.result_rows.items():
            value = values.get(key, "-")
            label.setText(f"{value:.3f}" if isinstance(value, float) else f"{value}")

    def _set_roi_from_view(self, roi: tuple) -> None:
        x, y, w, h = [float(value) for value in roi]
        blocked = [self.roi_x, self.roi_y, self.roi_w, self.roi_h]
        for widget in blocked:
            widget.blockSignals(True)
        self.roi_x.setValue(x)
        self.roi_y.setValue(y)
        self.roi_w.setValue(w)
        self.roi_h.setValue(h)
        for widget in blocked:
            widget.blockSignals(False)
        self._apply_detection_config()
        self.status.setText(f"ROI set from image: X {x:.1f}%, Y {y:.1f}%, W {w:.1f}%, H {h:.1f}%")

    def _set_roi_edit_mode(self, enabled: bool) -> None:
        for view in [self.rgb_view, self.depth_view, self.preprocessed_view, self.processed_view]:
            view.set_roi_edit_enabled(enabled)
        self.set_roi_toggle.setText("ROI ON" if enabled else "Set ROI")

    def _update_test_summary(self, values: dict[str, float | bool | str]) -> None:
        ok = bool(values.get("part_ok", False))
        score = values.get("vision_score", 0.0)
        pixel_u = values.get("vision_pixel_u", 0.0)
        pixel_v = values.get("vision_pixel_v", 0.0)
        depth = values.get("vision_depth_mm", 0.0)
        x = values.get("vision_x", 0.0)
        y = values.get("vision_y", 0.0)
        z = values.get("vision_z", 0.0)
        self.test_summary.setText(
            f"Result: {'OK' if ok else 'NG'} | score {float(score):.3f} | pixel ({float(pixel_u):.1f}, {float(pixel_v):.1f}) | "
            f"depth {float(depth):.1f} mm | robot ({float(x):.1f}, {float(y):.1f}, {float(z):.1f})"
        )

    def _toggle_live(self, checked: bool) -> None:
        if checked and not self._camera_connected():
            result = self.devices.connect_realsense()
            if not result.success:
                self.status.setText(result.message)
                self.live_toggle.setChecked(False)
                self.connect_toggle.setChecked(False)
                self._update_connection_ui()
                return
            self.connect_toggle.setChecked(True)
        if checked:
            self.live_timer.start()
            self._live_tick()
        else:
            self.live_timer.stop()
        self._update_connection_ui()

    def _toggle_calibration_live(self, checked: bool) -> None:
        self.live_toggle.setChecked(checked)
        self._toggle_live(checked)

    def _live_tick(self) -> None:
        if self.auto_detect_check.isChecked():
            self._detect()
            return
        self._capture(update_status=False)

    def _camera_connected(self) -> bool:
        return bool(getattr(self.devices.camera3d, "connected", False))

    def _update_connection_ui(self) -> None:
        connected = self._camera_connected()
        self.connect_toggle.setChecked(connected)
        self.connect_toggle.setText("D405 ON" if connected else "D405 OFF")
        self.connect_toggle.setProperty("class", "primary" if connected else "")
        self.live_toggle.setText("Live ON" if self.live_timer.isActive() else "Live")
        if hasattr(self, "calibration_live_toggle"):
            self.calibration_live_toggle.blockSignals(True)
            self.calibration_live_toggle.setChecked(self.live_timer.isActive())
            self.calibration_live_toggle.setText("Live ON" if self.live_timer.isActive() else "Live Preview")
            self.calibration_live_toggle.blockSignals(False)
        self.connect_toggle.style().unpolish(self.connect_toggle)
        self.connect_toggle.style().polish(self.connect_toggle)

    def _on_detection_setup_changed(self, *_args) -> None:
        self._sync_detection_controls()
        self._apply_detection_config()

    def _sync_detection_controls(self) -> None:
        method = self.method_combo.currentText()
        self.method_label.setText(method)
        visible_parameters = set(DETECTION_ALGORITHMS.get(method, {}).get("parameters", ()))
        for name, widgets in self.detection_parameter_widgets.items():
            visible = name in visible_parameters
            for widget in widgets:
                widget.setVisible(visible)

    def _save_vision_settings(self) -> None:
        default_path = str(Path.cwd() / "vision_settings.json")
        path, _ = QFileDialog.getSaveFileName(self, "Save Vision Settings", default_path, "Vision Settings (*.json)")
        if not path:
            return
        data = self._vision_settings_to_dict()
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
        self.status.setText(f"Vision settings saved: {path}")

    def _load_vision_settings(self) -> None:
        default_path = str(Path.cwd())
        path, _ = QFileDialog.getOpenFileName(self, "Load Vision Settings", default_path, "Vision Settings (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            self._apply_vision_settings_dict(data)
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.status.setText(f"Vision settings load failed: {exc}")
            return
        self.status.setText(f"Vision settings loaded: {path}")

    def set_project_profiles(self, vision: dict | None, calibration: dict | None) -> None:
        self.vision_store = normalize_vision_store(vision)
        self.calibration_store = normalize_calibration_store(calibration)
        self._refresh_project_profile_combos()
        self._load_selected_vision_profile()
        self._load_selected_calibration_profile()

    def project_profiles(self) -> tuple[dict, dict]:
        self._save_current_vision_profile(silent=True)
        self._save_current_calibration_profile(silent=True)
        return normalize_vision_store(self.vision_store), normalize_calibration_store(self.calibration_store)

    def profile_names(self) -> tuple[list[str], list[str]]:
        return profile_names(self.vision_store), profile_names(self.calibration_store)

    def _refresh_project_profile_combos(self) -> None:
        self._syncing_profiles = True
        try:
            for combo, store in [
                (getattr(self, "vision_profile_combo", None), self.vision_store),
                (getattr(self, "calibration_profile_combo", None), self.calibration_store),
            ]:
                if combo is None:
                    continue
                active = str(store.get("active", ""))
                names = profile_names(store)
                combo.blockSignals(True)
                combo.clear()
                combo.addItems(names)
                if active:
                    combo.setCurrentText(active)
                combo.blockSignals(False)
        finally:
            self._syncing_profiles = False

    def _on_vision_profile_selected(self, name: str) -> None:
        if self._syncing_profiles or name not in profile_names(self.vision_store):
            return
        self._request_load_selected_vision_profile()

    def _on_calibration_profile_selected(self, name: str) -> None:
        if self._syncing_profiles or name not in profile_names(self.calibration_store):
            return
        self._request_load_selected_calibration_profile()

    def _request_load_selected_vision_profile(self) -> None:
        name = self.vision_profile_combo.currentText().strip()
        if name not in profile_names(self.vision_store):
            self.status.setText(f"Vision profile not found: {name}")
            return
        if not self._confirm_discard_profile_changes("vision", name):
            self._revert_profile_combo(self.vision_profile_combo, self.vision_store)
            return
        self._load_selected_vision_profile()

    def _request_load_selected_calibration_profile(self) -> None:
        name = self.calibration_profile_combo.currentText().strip()
        if name not in profile_names(self.calibration_store):
            self.status.setText(f"Calibration profile not found: {name}")
            return
        if not self._confirm_discard_profile_changes("calibration", name):
            self._revert_profile_combo(self.calibration_profile_combo, self.calibration_store)
            return
        self._load_selected_calibration_profile()

    def _load_selected_vision_profile(self) -> None:
        profile = find_profile(self.vision_store, self.vision_profile_combo.currentText() if hasattr(self, "vision_profile_combo") else None)
        if not profile:
            return
        self.vision_store["active"] = profile["name"]
        self._apply_detection_settings_dict(profile.get("detection", {}))
        self.status.setText(f"Vision profile loaded: {profile['name']}")
        self.profiles_changed.emit()

    def _load_selected_calibration_profile(self) -> None:
        profile = find_profile(self.calibration_store, self.calibration_profile_combo.currentText() if hasattr(self, "calibration_profile_combo") else None)
        if not profile:
            return
        self.calibration_store["active"] = profile["name"]
        self._apply_calibration_settings_dict(profile.get("calibration", {}))
        self.status.setText(f"Calibration profile loaded: {profile['name']}")
        self.profiles_changed.emit()

    def _save_current_vision_profile(self, silent: bool = False) -> None:
        name = self.vision_profile_combo.currentText().strip() if hasattr(self, "vision_profile_combo") else DEFAULT_VISION_PROFILE
        if not name:
            name = DEFAULT_VISION_PROFILE
        if not silent and self._profile_exists(self.vision_store, name):
            existing = find_profile(self.vision_store, name) or {}
            if self._payload_changed(existing.get("detection", {}), self._detection_settings_to_dict()):
                if not self._confirm_save_profile_changes("Vision", name):
                    self.status.setText(f"Vision profile save canceled: {name}")
                    return
        self.vision_store = replace_profile(self.vision_store, "detection", name, self._detection_settings_to_dict())
        self._refresh_project_profile_combos()
        if not silent:
            self.status.setText(f"Vision profile saved: {name}")
            self.profiles_changed.emit()

    def _save_current_calibration_profile(self, silent: bool = False) -> None:
        name = self.calibration_profile_combo.currentText().strip() if hasattr(self, "calibration_profile_combo") else DEFAULT_CALIBRATION_PROFILE
        if not name:
            name = DEFAULT_CALIBRATION_PROFILE
        if not silent and self._profile_exists(self.calibration_store, name):
            existing = find_profile(self.calibration_store, name) or {}
            if self._payload_changed(existing.get("calibration", {}), self._calibration_settings_to_dict()):
                if not self._confirm_save_profile_changes("Calibration", name):
                    self.status.setText(f"Calibration profile save canceled: {name}")
                    return
        self.calibration_store = replace_profile(self.calibration_store, "calibration", name, self._calibration_settings_to_dict())
        self._refresh_project_profile_combos()
        if not silent:
            self.status.setText(f"Calibration profile saved: {name}")
            self.profiles_changed.emit()

    def _confirm_save_profile_changes(self, title: str, name: str) -> bool:
        result = QMessageBox.question(
            self,
            f"Save {title} Profile",
            f"'{name}' profile has changes compared with the saved version.\nSave these changes?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )
        return result == QMessageBox.StandardButton.Yes

    def _confirm_discard_profile_changes(self, profile_type: str, target_name: str) -> bool:
        store = self.vision_store if profile_type == "vision" else self.calibration_store
        active_name = str(store.get("active", "")).strip()
        if not active_name:
            return True
        active = find_profile(store, active_name) or {}
        if profile_type == "vision":
            changed = self._payload_changed(active.get("detection", {}), self._detection_settings_to_dict())
            title = "Load Vision Profile"
        else:
            changed = self._payload_changed(active.get("calibration", {}), self._calibration_settings_to_dict())
            title = "Load Calibration Profile"
        if not changed:
            return True
        result = QMessageBox.question(
            self,
            title,
            f"Current unsaved changes will be discarded.\nLoad '{target_name}' profile?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return result == QMessageBox.StandardButton.Yes

    def _revert_profile_combo(self, combo: QComboBox, store: dict) -> None:
        combo.blockSignals(True)
        combo.setCurrentText(str(store.get("active", "")))
        combo.blockSignals(False)

    def _profile_exists(self, store: dict, name: str) -> bool:
        return name in profile_names(store)

    def _payload_changed(self, saved: object, current: object) -> bool:
        return not self._payload_equal(saved, current)

    def _payload_equal(self, left: object, right: object) -> bool:
        if isinstance(left, dict) and isinstance(right, dict):
            keys = set(left.keys()) | set(right.keys())
            return all(self._payload_equal(left.get(key), right.get(key)) for key in keys)
        if isinstance(left, list) and isinstance(right, list):
            if len(left) != len(right):
                return False
            return all(self._payload_equal(a, b) for a, b in zip(left, right))
        if isinstance(left, (int, float)) or isinstance(right, (int, float)):
            try:
                return abs(float(left) - float(right)) <= 0.002
            except (TypeError, ValueError):
                return False
        return str(left) == str(right)

    def _vision_settings_to_dict(self) -> dict:
        return {
            "version": 2,
            "detection": self._detection_settings_to_dict(),
            "calibration": self._calibration_settings_to_dict(),
        }

    def _detection_settings_to_dict(self) -> dict:
        return {
            "method": self.method_combo.currentText(),
            "threshold": self.threshold.value() / 255.0,
            "min_area": self.min_area.value(),
            "max_area": self.max_area.value(),
            "blob_polarity": self.blob_polarity.currentText(),
            "blur_kernel": self.blur_kernel.value(),
            "open_iterations": self.open_iterations.value(),
            "close_iterations": self.close_iterations.value(),
            "min_circularity": self.min_circularity.value(),
            "roi": {
                "x_percent": self.roi_x.value(),
                "y_percent": self.roi_y.value(),
                "w_percent": self.roi_w.value(),
                "h_percent": self.roi_h.value(),
            },
        }

    def _calibration_settings_to_dict(self) -> dict:
        return {
            "enabled": self.calibrated_check.isChecked(),
            "camera_to_robot": list(self.camera_to_robot_matrix),
            "method": self.calibration_method_combo.currentText(),
            "checkerboard": {
                "columns": int(self.checkerboard_cols.value()),
                "rows": int(self.checkerboard_rows.value()),
                "square_size_mm": self.checkerboard_square.value(),
            },
            "pairs": [
                {"camera": list(camera_point), "robot": list(robot_point), "type": self._pair_source(index)}
                for index, (camera_point, robot_point) in enumerate(self.calibration_pairs)
            ],
            "robot_poses": [
                {"x": pose[0], "y": pose[1], "z": pose[2], "rz": pose[3], "source": pose[4]}
                for pose in self.saved_robot_poses
            ],
        }

    def _apply_vision_settings_dict(self, data: dict) -> None:
        self._apply_detection_settings_dict(data["detection"])
        self._apply_calibration_settings_dict(data.get("calibration", {}))

    def _apply_detection_settings_dict(self, detection: dict) -> None:
        roi = detection.get("roi", {})
        method = str(detection.get("method", "Blob"))
        if method not in DETECTION_ALGORITHMS:
            method = "Blob"
        self.method_combo.setCurrentText(method)
        self.threshold.setValue(self._settings_threshold_to_ui(float(detection.get("threshold", 140.0))))
        self.min_area.setValue(float(detection.get("min_area", 80.0)))
        self.max_area.setValue(float(detection.get("max_area", 20000.0)))
        self.blob_polarity.setCurrentText(str(detection.get("blob_polarity", "Auto")))
        self.blur_kernel.setValue(float(detection.get("blur_kernel", 5.0)))
        self.open_iterations.setValue(float(detection.get("open_iterations", 0.0)))
        self.close_iterations.setValue(float(detection.get("close_iterations", 0.0)))
        self.min_circularity.setValue(float(detection.get("min_circularity", 0.0)))
        self.roi_x.setValue(float(roi.get("x_percent", 35.0)))
        self.roi_y.setValue(float(roi.get("y_percent", 30.0)))
        self.roi_w.setValue(float(roi.get("w_percent", 30.0)))
        self.roi_h.setValue(float(roi.get("h_percent", 40.0)))
        self._sync_detection_controls()
        self._apply_detection_config()

    def _apply_calibration_settings_dict(self, calibration: dict) -> None:
        matrix = calibration.get("camera_to_robot", CalibrationConfig.identity().camera_to_robot)
        self._set_camera_to_robot_matrix([float(value) for value in matrix])
        self.calibrated_check.setChecked(bool(calibration.get("enabled", False)))
        method = str(calibration.get("method", "Point Pair"))
        self.calibration_method_combo.setCurrentText(method if method in {"Point Pair", "Checkerboard"} else "Point Pair")
        checkerboard = calibration.get("checkerboard", {})
        self.checkerboard_cols.setValue(float(checkerboard.get("columns", 7)))
        self.checkerboard_rows.setValue(float(checkerboard.get("rows", 6)))
        self.checkerboard_square.setValue(float(checkerboard.get("square_size_mm", 20.0)))
        self.calibration_pairs = []
        self.calibration_pair_sources = []
        self.saved_robot_poses = []
        for pose in calibration.get("robot_poses", []):
            self.saved_robot_poses.append(
                (
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("z", 0.0)),
                    float(pose.get("rz", 0.0)),
                    str(pose.get("source", "Loaded")),
                )
            )
        for pair in calibration.get("pairs", []):
            camera = pair.get("camera", [])
            robot = pair.get("robot", [])
            if len(camera) == 3 and len(robot) >= 3:
                robot_rz = float(robot[3]) if len(robot) >= 4 else 0.0
                self.calibration_pairs.append(
                    (
                        (float(camera[0]), float(camera[1]), float(camera[2])),
                        (float(robot[0]), float(robot[1]), float(robot[2]), robot_rz),
                    )
                )
                self.calibration_pair_sources.append(str(pair.get("type", "Point Pair")))
        self._refresh_robot_pose_table()
        self._refresh_pair_table()
        self._sync_calibration_method()
        self._apply_calibration()

    def _settings_threshold_to_ui(self, value: float) -> float:
        if 0.0 <= value <= 1.0:
            return value * 255.0
        return max(0.0, min(255.0, value))

    def _detect_calibration_target(self) -> None:
        if self.calibration_method_combo.currentText() == "Checkerboard":
            self._detect_checkerboard_calibration_target()
        else:
            self._detect()
        self._update_calibration_status()

    def _detect_checkerboard_calibration_target(self) -> None:
        self._apply_calibration()
        detect_checkerboard = getattr(self.devices.camera2d, "detect_checkerboard", None)
        if not callable(detect_checkerboard):
            self.status.setText("Current camera driver does not support checkerboard detection")
            return
        values = detection_to_variables(detect_checkerboard(self._checkerboard_config()))
        self.last_detection = values
        self.status.setText(str(values.get("vision_message", "Checkerboard detect complete")))
        self._update_test_summary(values)
        self._update_result_rows(values)
        self._refresh_views()

    def _checkerboard_config(self) -> CheckerboardConfig:
        return CheckerboardConfig(
            columns=int(self.checkerboard_cols.value()),
            rows=int(self.checkerboard_rows.value()),
            square_size_mm=float(self.checkerboard_square.value()),
        )

    def _sync_calibration_method(self, *_args) -> None:
        is_checkerboard = self.calibration_method_combo.currentText() == "Checkerboard"
        self.checkerboard_box.setVisible(is_checkerboard)
        if hasattr(self, "calibration_camera_box"):
            self.calibration_camera_box.setTitle("1 Detect Checkerboard Image" if is_checkerboard else "1 Detect Point Image")
        if hasattr(self, "calibration_view"):
            self.calibration_view.title = "1 Detect Checkerboard / Result" if is_checkerboard else "1 Detect Point / Result"
        if hasattr(self, "calibration_pair_view"):
            self.calibration_pair_view.title = "3 Added Checkerboard Sample" if is_checkerboard else "3 Added Pair Snapshot"
        if hasattr(self, "detect_target_button"):
            self.detect_target_button.setText("1 Detect CB" if is_checkerboard else "1 Detect Point")
        if hasattr(self, "add_pair_button"):
            self.add_pair_button.setText("3 Add Sample" if is_checkerboard else "3 Add Pair")
        if hasattr(self, "table_tabs") and hasattr(self, "pair_tab_index"):
            self.table_tabs.setTabText(self.pair_tab_index, "Checkerboard Samples" if is_checkerboard else "Point Pairs")
            self.table_tabs.setCurrentIndex(self.pair_tab_index)
        self._update_calibration_status()

    def _open_calibration_manual(self) -> None:
        if self.calibration_manual_dialog is not None and self.calibration_manual_dialog.isVisible():
            self.calibration_manual_dialog.raise_()
            self.calibration_manual_dialog.activateWindow()
            return
        dialog = QDialog(self)
        method = self.calibration_method_combo.currentText()
        dialog.setWindowTitle(f"{method} Calibration Manual")
        dialog.resize(760, 640)
        dialog.setMinimumSize(620, 480)
        dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        text = QTextBrowser()
        text.setOpenExternalLinks(False)
        text.setStyleSheet(
            """
            QTextBrowser {
                background: #0f172a;
                color: #e5edf7;
                border: 1px solid #33445f;
                border-radius: 6px;
                padding: 8px;
            }
            QScrollBar:vertical {
                background: #111827;
                width: 12px;
            }
            QScrollBar::handle:vertical {
                background: #33445f;
                border-radius: 4px;
            }
            """
        )
        text.setHtml(calibration_manual_html(method))
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(text)
        layout.addWidget(buttons)
        dialog.finished.connect(lambda *_: setattr(self, "calibration_manual_dialog", None))
        self.calibration_manual_dialog = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self.status.setText(f"{method} calibration manual opened")

    def _read_calibration_robot_pose(self) -> None:
        pose = self.devices.robot_pose()
        self._set_calibration_robot_inputs(float(pose.x), float(pose.y), float(pose.z), float(pose.r), "GetPose")
        self._update_calibration_status()
        self.status.setText(
            f"Robot pose captured for calibration: X {pose.x:.3f}, Y {pose.y:.3f}, Z {pose.z:.3f}, RZ {pose.r:.3f}"
        )

    def _set_calibration_robot_inputs(self, x: float, y: float, z: float, rz: float, source: str = "Manual") -> None:
        for spin, value in [(self.cal_robot_x, x), (self.cal_robot_y, y), (self.cal_robot_z, z), (self.cal_robot_rz, rz)]:
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        self.current_calibration_robot = (x, y, z, rz)
        self.current_calibration_robot_source = source

    def _on_calibration_robot_input_changed(self, *_args) -> None:
        self.current_calibration_robot = (
            float(self.cal_robot_x.value()),
            float(self.cal_robot_y.value()),
            float(self.cal_robot_z.value()),
            float(self.cal_robot_rz.value()),
        )
        self.current_calibration_robot_source = "Manual"
        self._update_calibration_status()

    def _save_calibration_robot_pose(self) -> None:
        pose = (
            float(self.cal_robot_x.value()),
            float(self.cal_robot_y.value()),
            float(self.cal_robot_z.value()),
            float(self.cal_robot_rz.value()),
        )
        self.current_calibration_robot = pose
        self.saved_robot_poses.append((*pose, self.current_calibration_robot_source))
        self._refresh_robot_pose_table()
        self._update_calibration_status()
        self.status.setText(f"Robot pose saved: {len(self.saved_robot_poses)}")

    def _use_selected_robot_pose(self) -> None:
        row = self.robot_pose_table.currentRow()
        if row < 0 or row >= len(self.saved_robot_poses):
            self.status.setText("Select a saved robot pose first")
            return
        x, y, z, rz, source = self.saved_robot_poses[row]
        self._set_calibration_robot_inputs(x, y, z, rz, source)
        self._update_calibration_status()
        self.status.setText(f"Saved robot pose loaded: {row + 1}")

    def _refresh_robot_pose_table(self) -> None:
        if not hasattr(self, "robot_pose_table"):
            return
        self.robot_pose_table.setRowCount(len(self.saved_robot_poses))
        for row, pose in enumerate(self.saved_robot_poses):
            values = [row + 1, *pose[:4], pose[4]]
            for col, value in enumerate(values):
                self.robot_pose_table.setItem(row, col, QTableWidgetItem(self._pair_table_text(value)))

    def _delete_selected_robot_pose(self) -> None:
        row = self.robot_pose_table.currentRow()
        if row < 0 or row >= len(self.saved_robot_poses):
            self.status.setText("Select a saved robot pose to remove")
            return
        del self.saved_robot_poses[row]
        self._refresh_robot_pose_table()
        self.status.setText(f"Robot pose removed: {row + 1}")

    def _add_calibration_pair(self) -> None:
        if not self.last_detection.get("part_ok"):
            self.status.setText("Detect the calibration target before adding a pair")
            return
        try:
            camera_point = (
                float(self.last_detection["vision_camera_x_mm"]),
                float(self.last_detection["vision_camera_y_mm"]),
                float(self.last_detection["vision_camera_z_mm"]),
            )
        except (KeyError, TypeError, ValueError):
            self.status.setText("Detection does not include a camera XYZ point")
            return
        if self.current_calibration_robot is None:
            pose = self.devices.robot_pose()
            self._set_calibration_robot_inputs(float(pose.x), float(pose.y), float(pose.z), float(pose.r), "GetPose")
        robot_point = self.current_calibration_robot
        self.calibration_pairs.append((camera_point, robot_point))
        self.calibration_pair_sources.append(self.calibration_method_combo.currentText())
        self._refresh_added_pair_image()
        self._refresh_pair_table()
        self._update_calibration_status()
        self.status.setText(f"Calibration pair added: {len(self.calibration_pairs)}")

    def _clear_calibration_pairs(self) -> None:
        self.calibration_pairs.clear()
        self.calibration_pair_sources.clear()
        self.current_calibration_robot = None
        if hasattr(self, "calibration_pair_view"):
            self.calibration_pair_view.set_overlay(None, None)
            self.calibration_pair_view.set_frame(None)
        self._refresh_pair_table()
        self._update_calibration_status()
        self.status.setText("Calibration pairs cleared")

    def _delete_selected_calibration_pair(self) -> None:
        row = self.pair_table.currentRow()
        if row < 0 or row >= len(self.calibration_pairs):
            self.status.setText("Select a calibration pair to remove")
            return
        del self.calibration_pairs[row]
        if row < len(self.calibration_pair_sources):
            del self.calibration_pair_sources[row]
        self._refresh_pair_table()
        self._update_calibration_status()
        self.status.setText(f"Calibration pair removed: {row + 1}")

    def _refresh_added_pair_image(self) -> None:
        if not hasattr(self, "calibration_pair_view"):
            return
        marker = None
        try:
            if self.last_detection.get("part_ok"):
                marker = (float(self.last_detection["vision_pixel_u"]), float(self.last_detection["vision_pixel_v"]))
        except (KeyError, TypeError, ValueError):
            marker = None
        processed_frame = getattr(self.devices.camera2d, "result_preview_frame", lambda: None)()
        color_frame = getattr(self.devices.camera2d, "color_frame", lambda: None)()
        self.calibration_pair_view.set_overlay(None, marker)
        self.calibration_pair_view.set_frame(processed_frame if processed_frame is not None else color_frame)

    def _refresh_pair_table(self) -> None:
        self.pair_table.setRowCount(len(self.calibration_pairs))
        errors = self._calibration_errors()
        for row, (camera_point, robot_point) in enumerate(self.calibration_pairs):
            values = [row + 1, self._pair_source(row), *camera_point, *robot_point, errors[row] if row < len(errors) else None]
            for col, value in enumerate(values):
                text = self._pair_table_text(value)
                self.pair_table.setItem(row, col, QTableWidgetItem(text))

    def _pair_source(self, index: int) -> str:
        if 0 <= index < len(self.calibration_pair_sources):
            return self.calibration_pair_sources[index]
        return "Point Pair"

    def _pair_table_text(self, value: object) -> str:
        if value is None:
            return "-"
        if isinstance(value, int):
            return str(value)
        if isinstance(value, str):
            return value
        return f"{float(value):.3f}"

    def _solve_calibration_matrix(self) -> None:
        if len(self.calibration_pairs) < 3:
            self.status.setText("At least 3 calibration pairs are required")
            self._update_calibration_status()
            return
        camera_points = [pair[0] for pair in self.calibration_pairs]
        robot_points = [robot_xyz(pair[1]) for pair in self.calibration_pairs]
        matrix = solve_camera_to_robot_matrix(camera_points, robot_points)
        if matrix is None:
            self.status.setText("Calibration points need more spatial variation")
            self._update_calibration_status()
            return
        self._set_camera_to_robot_matrix(matrix)
        self.calibrated_check.setChecked(True)
        self._apply_calibration()
        self._refresh_pair_table()
        self._update_calibration_status()
        self.status.setText("Camera-to-robot calibration matrix solved and applied")

    def _update_calibration_status(self) -> None:
        count = len(self.calibration_pairs)
        if hasattr(self, "calibration_pair_count_label"):
            self.calibration_pair_count_label.setText(f"Pairs: {count}")
        if hasattr(self, "calibration_camera_label"):
            self.calibration_camera_label.setText(f"Camera XYZ: {self._last_camera_point_text()}")
        if hasattr(self, "calibration_robot_label"):
            self.calibration_robot_label.setText(f"Robot XYZ/RZ: {self._robot_point_text(self.current_calibration_robot)}")
        errors = self._calibration_errors()
        if errors:
            mean_error = sum(errors) / len(errors)
            max_error = max(errors)
            quality = f"Quality: mean {mean_error:.3f} mm / max {max_error:.3f} mm"
        else:
            quality = "Quality: solve matrix to calculate error"
        if hasattr(self, "calibration_quality_label"):
            self.calibration_quality_label.setText(quality)
        if not hasattr(self, "calibration_status_label"):
            return
        pair_word = "samples" if self.calibration_method_combo.currentText() == "Checkerboard" else "point pairs"
        if count < 3:
            self.calibration_status_label.setText(f"Need 3 or more {pair_word}")
            self.calibration_status_label.setProperty("class", "warn")
        elif not has_spatial_variation([pair[0] for pair in self.calibration_pairs]):
            self.calibration_status_label.setText("Need more spatial variation between camera points")
            self.calibration_status_label.setProperty("class", "warn")
        elif self.calibrated_check.isChecked():
            self.calibration_status_label.setText("Calibration applied")
            self.calibration_status_label.setProperty("class", "ok")
        else:
            self.calibration_status_label.setText("Ready to solve calibration")
            self.calibration_status_label.setProperty("class", "ok")
        self.calibration_status_label.style().unpolish(self.calibration_status_label)
        self.calibration_status_label.style().polish(self.calibration_status_label)

    def _last_camera_point_text(self) -> str:
        try:
            point = (
                float(self.last_detection["vision_camera_x_mm"]),
                float(self.last_detection["vision_camera_y_mm"]),
                float(self.last_detection["vision_camera_z_mm"]),
            )
        except (KeyError, TypeError, ValueError):
            return "-"
        return self._point_text(point)

    def _point_text(self, point: tuple[float, float, float] | None) -> str:
        if point is None:
            return "-"
        return f"X {point[0]:.3f}, Y {point[1]:.3f}, Z {point[2]:.3f}"

    def _robot_point_text(self, point: tuple[float, float, float, float] | None) -> str:
        if point is None:
            return "-"
        return f"X {point[0]:.3f}, Y {point[1]:.3f}, Z {point[2]:.3f}, RZ {point[3]:.3f}"

    def _calibration_errors(self) -> list[float]:
        if not self.calibrated_check.isChecked() or not self.calibration_pairs:
            return []
        return calibration_errors(self.camera_to_robot_matrix, self.calibration_pairs)
