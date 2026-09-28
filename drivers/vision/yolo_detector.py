from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class YoloDetection:
    bbox: tuple[int, int, int, int]
    confidence: float
    class_id: int
    class_name: str
    mask: np.ndarray | None
    center_px: tuple[float, float]
    angle_deg: float
    area: float


class YoloSegDetector:
    """Small wrapper around Ultralytics YOLO for runtime detection.

    The main app only needs a center point and an object orientation. Keeping
    this isolated avoids coupling the RealSense driver to the training utility.
    """

    def __init__(self) -> None:
        self._model: Any = None
        self._model_path = ""
        self._load_error = ""

    @property
    def load_error(self) -> str:
        return self._load_error

    def detect(
        self,
        image_rgb: np.ndarray,
        model_path: str,
        confidence: float,
        class_filter: str = "",
    ) -> list[YoloDetection]:
        if not self._ensure_model(model_path):
            return []
        try:
            import cv2  # type: ignore

            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            predictions = self._model.predict(source=image_bgr, conf=float(confidence), verbose=False, device="cpu")
        except Exception as exc:
            self._load_error = f"YOLO inference failed: {exc}"
            return []
        if not predictions:
            return []
        result = predictions[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        masks_data = getattr(result.masks, "data", None) if getattr(result, "masks", None) is not None else None
        names = getattr(result, "names", {}) or {}
        wanted = {item.strip().lower() for item in class_filter.split(",") if item.strip()}
        detections: list[YoloDetection] = []
        for index, box in enumerate(boxes):
            class_id = int(box.cls[0].item())
            class_name = str(names.get(class_id, class_id))
            if wanted and class_name.lower() not in wanted and str(class_id) not in wanted:
                continue
            bbox_values = box.xyxy[0].cpu().numpy().astype(int).tolist()
            x1, y1, x2, y2 = [int(value) for value in bbox_values]
            mask = None
            if masks_data is not None and index < len(masks_data):
                raw = masks_data[index].cpu().numpy()
                mask = (raw > 0.5).astype(np.uint8)
                if mask.shape[:2] != image_rgb.shape[:2]:
                    mask = cv2.resize(mask, (image_rgb.shape[1], image_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
            center, angle, area = self._shape_pose(mask, (x1, y1, x2, y2))
            detections.append(
                YoloDetection(
                    bbox=(x1, y1, x2, y2),
                    confidence=float(box.conf[0].item()),
                    class_id=class_id,
                    class_name=class_name,
                    mask=mask,
                    center_px=center,
                    angle_deg=angle,
                    area=area,
                )
            )
        return detections

    def draw_overlay(self, image_rgb: np.ndarray, detections: list[YoloDetection], selected: YoloDetection | None) -> np.ndarray:
        try:
            import cv2  # type: ignore
        except Exception:
            return image_rgb.copy()
        overlay = image_rgb.copy()
        for detection in detections:
            color = (34, 197, 94) if detection is selected else (96, 165, 250)
            if detection.mask is not None:
                mask_layer = np.zeros_like(overlay)
                mask_layer[:, :, 1] = (detection.mask > 0).astype(np.uint8) * 170
                overlay = cv2.addWeighted(overlay, 1.0, mask_layer, 0.38, 0)
            x1, y1, x2, y2 = detection.bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cx, cy = detection.center_px
            self._draw_angle_axis(cv2, overlay, int(round(cx)), int(round(cy)), detection.angle_deg, color)
            label = f"{detection.class_name} {detection.confidence:.2f} RZ {detection.angle_deg:.1f}"
            cv2.putText(overlay, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        return overlay

    def _ensure_model(self, model_path: str) -> bool:
        path = str(Path(model_path).expanduser())
        if not path:
            self._load_error = "YOLO model path is empty"
            self._model = None
            self._model_path = ""
            return False
        if not Path(path).exists():
            self._load_error = f"YOLO model not found: {path}"
            self._model = None
            self._model_path = ""
            return False
        if self._model is not None and self._model_path == path:
            return True
        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(path)
            self._model_path = path
            self._load_error = ""
            return True
        except Exception as exc:
            self._model = None
            self._model_path = ""
            self._load_error = f"ultralytics YOLO load failed: {exc}"
            return False

    def _shape_pose(self, mask: np.ndarray | None, bbox: tuple[int, int, int, int]) -> tuple[tuple[float, float], float, float]:
        try:
            import cv2  # type: ignore
        except Exception:
            x1, y1, x2, y2 = bbox
            return ((x1 + x2) * 0.5, (y1 + y2) * 0.5), 0.0, float(max(0, x2 - x1) * max(0, y2 - y1))
        if mask is not None and np.count_nonzero(mask) >= 5:
            contours, _ = cv2.findContours((mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                contour = max(contours, key=cv2.contourArea)
                moments = cv2.moments(contour)
                if abs(moments["m00"]) > 1e-9:
                    center = (float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"]))
                else:
                    center = tuple(float(value) for value in cv2.minAreaRect(contour)[0])
                rect = cv2.minAreaRect(contour)
                return center, self._normalize_rect_angle(rect), float(cv2.contourArea(contour))
        x1, y1, x2, y2 = bbox
        center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        angle = 0.0 if width >= height else 90.0
        return center, angle, float(width * height)

    def _normalize_rect_angle(self, rect: Any) -> float:
        (_center_x, _center_y), (width, height), raw_angle = rect
        angle = float(raw_angle)
        if width < height:
            angle += 90.0
        while angle > 90.0:
            angle -= 180.0
        while angle <= -90.0:
            angle += 180.0
        return angle

    def _draw_angle_axis(self, cv2: Any, image: np.ndarray, cx: int, cy: int, angle_deg: float, color: tuple[int, int, int]) -> None:
        radians = np.deg2rad(angle_deg)
        length = 34
        dx = int(round(np.cos(radians) * length))
        dy = int(round(np.sin(radians) * length))
        cv2.drawMarker(image, (cx, cy), (239, 68, 68), cv2.MARKER_CROSS, 18, 2)
        cv2.line(image, (cx - dx, cy - dy), (cx + dx, cy + dy), color, 2)
