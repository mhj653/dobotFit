from __future__ import annotations

from typing import Any

import numpy as np

from core.features import yolo_enabled
from core.models import Result
from drivers.camera.base import CalibrationConfig, CameraFrame, CheckerboardConfig, DetectionConfig, VisionResult


class RealSenseD405Camera:
    """Intel RealSense D405 RGB-D camera driver.

    The first implementation provides live RGB/depth capture and a center-ROI
    3D locate result. Robot-coordinate accuracy still requires camera-to-robot
    calibration; until then, vision_x/y/z are bounded test coordinates derived
    from camera XYZ so the sequence flow can be verified safely in simulation.
    """

    def __init__(self, serial: str = "", width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.connected = False
        self.name = "RealSense D405"
        self._rs: Any = None
        self._pipeline: Any = None
        self._align: Any = None
        self._profile: Any = None
        self._depth_scale = 0.001
        self._intrinsics: Any = None
        self._last_color: np.ndarray | None = None
        self._last_depth: np.ndarray | None = None
        self._last_preprocessed: np.ndarray | None = None
        self._last_result: np.ndarray | None = None
        self._last_processed: np.ndarray | None = None
        self.detection_config = DetectionConfig()
        self.calibration = CalibrationConfig.identity()
        self._yolo_detector: Any | None = None
        self._detectors = {"Blob": self._detect_blob}
        if yolo_enabled():
            from drivers.vision.yolo_detector import YoloSegDetector

            self._yolo_detector = YoloSegDetector()
            self._detectors["YOLO Segmentation"] = self._detect_yolo_segmentation

    def configure_detection(self, config: DetectionConfig) -> None:
        self.detection_config = config

    def refresh_detection_preview(self) -> None:
        if self._last_color is None:
            return
        height, width = self._last_color.shape[:2]
        roi_x, roi_y, roi_w, roi_h = self.detection_config.roi_pixels(width, height)
        self._detect_pixel_in_roi(roi_x, roi_y, roi_w, roi_h)

    def configure_calibration(self, config: CalibrationConfig) -> None:
        self.calibration = config

    def connect(self) -> Result:
        try:
            import pyrealsense2 as rs  # type: ignore
        except Exception as exc:
            return Result.fail(f"pyrealsense2 is not installed: {exc}", "SDK_NOT_FOUND")

        if self.connected:
            return Result.ok(f"{self.name} already connected")
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
            profile = pipeline.start(config)
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = float(sensor.get_depth_scale())
            self._rs = rs
            self._pipeline = pipeline
            self._profile = profile
            self._align = rs.align(rs.stream.color)
            self.connected = True
            return Result.ok(f"{self.name} connected")
        except Exception as exc:
            self.connected = False
            self._pipeline = None
            return Result.fail(f"{self.name} connect failed: {exc}", "CAMERA_CONNECT_FAILED")

    def disconnect(self) -> Result:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self.connected = False
        self._pipeline = None
        self._align = None
        self._profile = None
        return Result.ok(f"{self.name} disconnected")

    def capture(self) -> Result:
        if not self.connected:
            result = self.connect()
            if not result.success:
                return result
        if self._pipeline is None or self._align is None:
            return Result.fail(f"{self.name} pipeline is not ready", "CAMERA_NOT_READY")
        try:
            last_color = None
            last_depth = None
            last_intrinsics = None
            for _ in range(8):
                frames = self._pipeline.wait_for_frames(3000)
                aligned = self._align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                color = np.asanyarray(color_frame.get_data()).copy()
                depth = np.asanyarray(depth_frame.get_data()).copy()
                last_color = color
                last_depth = depth
                last_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                if np.count_nonzero(depth) > 0:
                    break
            if last_color is None or last_depth is None or last_intrinsics is None:
                return Result.fail("RealSense frame timeout", "FRAME_TIMEOUT")
            self._last_color = last_color
            self._last_depth = last_depth
            self._last_preprocessed = last_color.copy()
            self._last_result = last_color.copy()
            self._last_processed = last_color.copy()
            self._intrinsics = last_intrinsics
            valid = int(np.count_nonzero(last_depth))
            return Result.ok(f"{self.name} RGB-D capture complete ({valid} depth pixels)")
        except Exception as exc:
            return Result.fail(f"{self.name} capture failed: {exc}", "CAPTURE_FAILED")

    def detect(self) -> dict[str, float | bool | str]:
        result = self.capture()
        if not result.success:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message=result.message).to_variables()
        if self._last_depth is None or self._intrinsics is None or self._rs is None:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message="No depth frame").to_variables()

        height, width = self._last_depth.shape
        roi_x, roi_y, roi_w, roi_h = self.detection_config.roi_pixels(width, height)
        cx, cy = roi_x + roi_w // 2, roi_y + roi_h // 2
        selected_pixel = self._detect_pixel_in_roi(roi_x, roi_y, roi_w, roi_h)
        if selected_pixel[0] is None or selected_pixel[1] is None:
            return VisionResult(
                False,
                300.0,
                0.0,
                180.0,
                0.0,
                pixel_u=float(cx),
                pixel_v=float(cy),
                message=selected_pixel[3],
            ).to_variables()
        else:
            px = int(selected_pixel[0])
            py = int(selected_pixel[1])
            score = float(selected_pixel[2])
            method_message = selected_pixel[3]
            object_rz = self._normalize_rz(float(selected_pixel[4]) + float(self.detection_config.angle_offset_deg))
        valid_mask = self._last_depth > 0
        x0, y0, x1, y1 = roi_x, roi_y, roi_x + roi_w, roi_y + roi_h
        if not valid_mask.any():
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, pixel_u=float(px), pixel_v=float(py), message="No valid depth pixels").to_variables()

        depth_px, depth_py = self._nearest_valid_depth_pixel(valid_mask, px, py, (x0, y0, x1, y1))
        if depth_px is None or depth_py is None:
            depth_px, depth_py = self._nearest_valid_depth_pixel(valid_mask, px, py, None)
        if depth_px is None or depth_py is None:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, pixel_u=float(px), pixel_v=float(py), message="No valid depth pixels").to_variables()
        px = depth_px
        py = depth_py
        roi_half = 8
        local = self._last_depth[max(0, py - roi_half) : min(height, py + roi_half + 1), max(0, px - roi_half) : min(width, px + roi_half + 1)]
        local_valid = local[local > 0]
        depth_m = float(np.median(local_valid if local_valid.size else [self._last_depth[py, px]]) * self._depth_scale)
        camera_point = self._rs.rs2_deproject_pixel_to_point(self._intrinsics, [float(px), float(py)], depth_m)
        camera_x_mm = float(camera_point[0] * 1000.0)
        camera_y_mm = float(camera_point[1] * 1000.0)
        camera_z_mm = float(camera_point[2] * 1000.0)

        if self.calibration.calibrated:
            robot_x, robot_y, robot_z = self._camera_to_robot(camera_x_mm, camera_y_mm, camera_z_mm)
        else:
            robot_x = max(80.0, min(440.0, 300.0 + camera_x_mm))
            robot_y = max(-440.0, min(440.0, camera_y_mm))
            robot_z = max(40.0, min(350.0, 180.0 + (camera_z_mm - 250.0) * 0.15))
        return VisionResult(
            True,
            robot_x,
            robot_y,
            robot_z,
            object_rz,
            score=score,
            pixel_u=float(px),
            pixel_v=float(py),
            depth_mm=camera_z_mm,
            camera_x_mm=camera_x_mm,
            camera_y_mm=camera_y_mm,
            camera_z_mm=camera_z_mm,
            calibrated=self.calibration.calibrated,
            message=f"RealSense {method_message}",
        ).to_variables()

    def detect_checkerboard(self, config: CheckerboardConfig) -> dict[str, float | bool | str]:
        result = self.capture()
        if not result.success:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message=result.message).to_variables()
        if self._last_color is None or self._last_depth is None or self._intrinsics is None or self._rs is None:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message="No RGB-D frame").to_variables()
        try:
            import cv2  # type: ignore
        except Exception as exc:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message=f"OpenCV is not available: {exc}").to_variables()

        columns = max(2, int(config.columns))
        rows = max(2, int(config.rows))
        gray = cv2.cvtColor(self._last_color, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (columns, rows), None)
        preview = self._last_color.copy()
        if not found or corners is None:
            self._last_preprocessed = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            self._last_result = preview
            self._last_processed = preview
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, message=f"Checkerboard not found ({columns}x{rows})").to_variables()

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(preview, (columns, rows), corners, found)
        center = corners.reshape(-1, 2).mean(axis=0)
        px = int(round(float(center[0])))
        py = int(round(float(center[1])))
        cv2.drawMarker(preview, (px, py), (239, 68, 68), cv2.MARKER_CROSS, 22, 2)
        self._last_preprocessed = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        self._last_result = preview
        self._last_processed = preview

        point = self._camera_point_from_pixel(px, py, None)
        if point is None:
            return VisionResult(False, 300.0, 0.0, 180.0, 0.0, pixel_u=float(px), pixel_v=float(py), message="No valid depth near checkerboard center").to_variables()
        camera_x_mm, camera_y_mm, camera_z_mm, depth_px, depth_py = point
        if self.calibration.calibrated:
            robot_x, robot_y, robot_z = self._camera_to_robot(camera_x_mm, camera_y_mm, camera_z_mm)
        else:
            robot_x = max(80.0, min(440.0, 300.0 + camera_x_mm))
            robot_y = max(-440.0, min(440.0, camera_y_mm))
            robot_z = max(40.0, min(350.0, 180.0 + (camera_z_mm - 250.0) * 0.15))
        score = min(1.0, float(len(corners)) / float(columns * rows))
        return VisionResult(
            True,
            robot_x,
            robot_y,
            robot_z,
            0.0,
            score=score,
            pixel_u=float(depth_px),
            pixel_v=float(depth_py),
            depth_mm=camera_z_mm,
            camera_x_mm=camera_x_mm,
            camera_y_mm=camera_y_mm,
            camera_z_mm=camera_z_mm,
            calibrated=self.calibration.calibrated,
            message=f"Checkerboard locate complete ({columns}x{rows}, square {config.square_size_mm:.1f} mm)",
        ).to_variables()

    def color_frame(self) -> CameraFrame | None:
        if self._last_color is None:
            return None
        height, width, channels = self._last_color.shape
        return CameraFrame(width, height, channels, self._last_color.tobytes(), "rgb8")

    def depth_preview_frame(self) -> CameraFrame | None:
        if self._last_depth is None:
            return None
        depth = self._last_depth.astype(np.float32) * self._depth_scale * 1000.0
        valid = depth[depth > 0]
        if valid.size:
            near = float(np.percentile(valid, 5))
            far = float(np.percentile(valid, 95))
            span = max(far - near, 1.0)
            normalized = np.clip((depth - near) / span, 0.0, 1.0)
        else:
            normalized = np.zeros_like(depth)
        gray = (normalized * 255.0).astype(np.uint8)
        rgb = np.stack([gray, np.clip(gray * 1.3, 0, 255).astype(np.uint8), 255 - gray], axis=2)
        height, width, channels = rgb.shape
        return CameraFrame(width, height, channels, rgb.tobytes(), "rgb8")

    def _camera_to_robot(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        matrix = self.calibration.camera_to_robot
        rx = matrix[0] * x_mm + matrix[1] * y_mm + matrix[2] * z_mm + matrix[3]
        ry = matrix[4] * x_mm + matrix[5] * y_mm + matrix[6] * z_mm + matrix[7]
        rz = matrix[8] * x_mm + matrix[9] * y_mm + matrix[10] * z_mm + matrix[11]
        return rx, ry, rz

    def _camera_point_from_pixel(
        self,
        px: int,
        py: int,
        bounds: tuple[int, int, int, int] | None,
    ) -> tuple[float, float, float, int, int] | None:
        if self._last_depth is None or self._intrinsics is None or self._rs is None:
            return None
        valid_mask = self._last_depth > 0
        if not valid_mask.any():
            return None
        depth_px, depth_py = self._nearest_valid_depth_pixel(valid_mask, px, py, bounds)
        if depth_px is None or depth_py is None:
            depth_px, depth_py = self._nearest_valid_depth_pixel(valid_mask, px, py, None)
        if depth_px is None or depth_py is None:
            return None
        height, width = self._last_depth.shape
        roi_half = 8
        local = self._last_depth[
            max(0, depth_py - roi_half) : min(height, depth_py + roi_half + 1),
            max(0, depth_px - roi_half) : min(width, depth_px + roi_half + 1),
        ]
        local_valid = local[local > 0]
        depth_m = float(np.median(local_valid if local_valid.size else [self._last_depth[depth_py, depth_px]]) * self._depth_scale)
        camera_point = self._rs.rs2_deproject_pixel_to_point(self._intrinsics, [float(depth_px), float(depth_py)], depth_m)
        return (
            float(camera_point[0] * 1000.0),
            float(camera_point[1] * 1000.0),
            float(camera_point[2] * 1000.0),
            int(depth_px),
            int(depth_py),
        )

    def _detect_pixel_in_roi(self, roi_x: int, roi_y: int, roi_w: int, roi_h: int) -> tuple[int | None, int | None, float, str, float]:
        method = self.detection_config.method
        if self._last_color is None:
            return None, None, 0.0, "No RGB frame for vision detection", 0.0
        try:
            import cv2  # type: ignore
        except Exception as exc:
            return None, None, 0.0, f"OpenCV is not available: {exc}", 0.0
        roi = self._last_color[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        detector = self._detectors.get(method)
        if detector is not None:
            return detector(cv2, gray, roi_x, roi_y)
        return None, None, 0.0, f"Unsupported detection method: {method}", 0.0

    def _detect_blob(self, cv2: Any, gray: np.ndarray, roi_x: int, roi_y: int) -> tuple[int | None, int | None, float, str, float]:
        threshold = int(max(0, min(255, self.detection_config.threshold * 255.0)))
        blur_kernel = max(0, int(self.detection_config.blur_kernel))
        if blur_kernel >= 3:
            if blur_kernel % 2 == 0:
                blur_kernel += 1
            working = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        else:
            working = gray
        polarity = self.detection_config.blob_polarity
        if polarity == "Bright":
            modes = [cv2.THRESH_BINARY]
        elif polarity == "Dark":
            modes = [cv2.THRESH_BINARY_INV]
        else:
            modes = [cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        open_iterations = max(0, int(self.detection_config.open_iterations))
        close_iterations = max(0, int(self.detection_config.close_iterations))
        candidates: list[tuple[float, float, float, float, float, np.ndarray]] = []
        binary_preview: np.ndarray | None = None
        result_preview = self._last_color.copy() if self._last_color is not None else cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        for mode in modes:
            _, binary = cv2.threshold(working, threshold, 255, mode)
            if open_iterations:
                binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=open_iterations)
            if close_iterations:
                binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
            if binary_preview is None:
                binary_preview = binary.copy()
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.detection_config.min_area or area > self.detection_config.max_area:
                    continue
                perimeter = float(cv2.arcLength(contour, True))
                circularity = 0.0 if perimeter <= 1e-9 else float(4.0 * np.pi * area / (perimeter * perimeter))
                if circularity < self.detection_config.min_circularity:
                    continue
                moments = cv2.moments(contour)
                if abs(moments["m00"]) < 1e-9:
                    continue
                px = float(moments["m10"] / moments["m00"]) + roi_x
                py = float(moments["m01"] / moments["m00"]) + roi_y
                angle = self._contour_angle_deg(cv2, contour)
                candidates.append((area, px, py, circularity, angle, binary.copy()))
                shifted = (contour + np.array([[[roi_x, roi_y]]], dtype=contour.dtype)).astype(np.int32)
                cv2.drawContours(result_preview, [shifted], -1, (34, 197, 94), 2)
        if binary_preview is None:
            binary_preview = np.zeros_like(gray)
        self._last_result = result_preview
        self._last_processed = result_preview
        if not candidates:
            self._last_preprocessed = self._compose_roi_preview(cv2.cvtColor(binary_preview, cv2.COLOR_GRAY2RGB), roi_x, roi_y)
            return None, None, 0.0, "Blob not found in ROI", 0.0
        area, px, py, circularity, angle, selected_binary = max(candidates, key=lambda item: item[0])
        self._last_preprocessed = self._compose_roi_preview(cv2.cvtColor(selected_binary, cv2.COLOR_GRAY2RGB), roi_x, roi_y)
        cv2.drawMarker(result_preview, (int(round(px)), int(round(py))), (239, 68, 68), cv2.MARKER_CROSS, 18, 2)
        self._draw_angle_axis(cv2, result_preview, int(round(px)), int(round(py)), angle)
        self._last_result = result_preview
        self._last_processed = result_preview
        score = min(1.0, area / max(self.detection_config.max_area, 1.0))
        return int(round(px)), int(round(py)), score, f"Blob locate complete (area {area:.1f}, circularity {circularity:.2f}, RZ {angle:.1f})", angle

    def _detect_yolo_segmentation(self, cv2: Any, gray: np.ndarray, roi_x: int, roi_y: int) -> tuple[int | None, int | None, float, str, float]:
        if self._last_color is None:
            return None, None, 0.0, "No RGB frame for YOLO detection", 0.0
        if self._yolo_detector is None:
            return None, None, 0.0, "YOLO feature is not enabled in this build", 0.0
        roi_h, roi_w = gray.shape[:2]
        roi_image = self._last_color[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
        detections = self._yolo_detector.detect(
            roi_image,
            self.detection_config.yolo_model_path,
            self.detection_config.threshold,
            self.detection_config.yolo_class_filter,
        )
        detections = [
            detection
            for detection in detections
            if self.detection_config.min_area <= detection.area <= self.detection_config.max_area
        ]
        if not detections:
            self._last_preprocessed = self._last_color.copy()
            self._last_result = self._last_color.copy()
            self._last_processed = self._last_result
            error = self._yolo_detector.load_error or "YOLO object not found in ROI"
            return None, None, 0.0, error, 0.0

        selected = self._select_yolo_detection(detections)
        result_preview = self._last_color.copy()
        roi_overlay = self._yolo_detector.draw_overlay(roi_image, detections, selected)
        result_preview[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w] = roi_overlay

        preprocessed = self._last_color.copy()
        if selected.mask is not None:
            mask_rgb = np.stack([(selected.mask > 0).astype(np.uint8) * 255] * 3, axis=2)
            preprocessed[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w] = mask_rgb
        else:
            preprocessed[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w] = roi_overlay

        center_x = selected.center_px[0] + roi_x
        center_y = selected.center_px[1] + roi_y
        self._draw_angle_axis(cv2, result_preview, int(round(center_x)), int(round(center_y)), selected.angle_deg)
        self._last_preprocessed = preprocessed
        self._last_result = result_preview
        self._last_processed = result_preview
        return (
            int(round(center_x)),
            int(round(center_y)),
            float(selected.confidence),
            f"YOLO locate complete ({selected.class_name}, conf {selected.confidence:.2f}, RZ {selected.angle_deg:.1f})",
            float(selected.angle_deg),
        )

    def _select_yolo_detection(self, detections: list[Any]) -> Any:
        return max(detections, key=lambda detection: (detection.confidence, detection.area))

    def _contour_angle_deg(self, cv2: Any, contour: np.ndarray) -> float:
        rect = cv2.minAreaRect(contour)
        (_center_x, _center_y), (width, height), raw_angle = rect
        angle = float(raw_angle)
        if width < height:
            angle += 90.0
        return self._normalize_rz(angle)

    def _draw_angle_axis(self, cv2: Any, image: np.ndarray, cx: int, cy: int, angle_deg: float) -> None:
        radians = np.deg2rad(angle_deg)
        length = 34
        dx = int(round(np.cos(radians) * length))
        dy = int(round(np.sin(radians) * length))
        cv2.line(image, (cx - dx, cy - dy), (cx + dx, cy + dy), (250, 204, 21), 2)

    def _normalize_rz(self, angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle

    def processed_preview_frame(self) -> CameraFrame | None:
        return self.result_preview_frame()

    def preprocessed_preview_frame(self) -> CameraFrame | None:
        if self._last_preprocessed is None:
            return self.color_frame()
        height, width, channels = self._last_preprocessed.shape
        return CameraFrame(width, height, channels, self._last_preprocessed.tobytes(), "rgb8")

    def result_preview_frame(self) -> CameraFrame | None:
        if self._last_result is None:
            return self.color_frame()
        height, width, channels = self._last_result.shape
        return CameraFrame(width, height, channels, self._last_result.tobytes(), "rgb8")

    def _compose_roi_preview(self, roi_preview: np.ndarray, roi_x: int, roi_y: int) -> np.ndarray:
        if self._last_color is None:
            return roi_preview.copy()
        canvas = self._last_color.copy()
        height, width = roi_preview.shape[:2]
        canvas[roi_y : roi_y + height, roi_x : roi_x + width] = roi_preview
        return canvas

    def _nearest_valid_depth_pixel(
        self,
        valid_mask: np.ndarray,
        target_x: int,
        target_y: int,
        bounds: tuple[int, int, int, int] | None,
    ) -> tuple[int | None, int | None]:
        if bounds is None:
            ys, xs = np.nonzero(valid_mask)
        else:
            x0, y0, x1, y1 = bounds
            local = valid_mask[y0:y1, x0:x1]
            if not local.any():
                return None, None
            ys, xs = np.nonzero(local)
            xs = xs + x0
            ys = ys + y0
        if xs.size == 0:
            return None, None
        distances = (xs - target_x) ** 2 + (ys - target_y) ** 2
        selected = int(np.argmin(distances))
        return int(xs[selected]), int(ys[selected])
