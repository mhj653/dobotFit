from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class CameraFrame:
    width: int
    height: int
    channels: int
    data: bytes
    pixel_format: str = "rgb8"


@dataclass
class DetectionConfig:
    method: str = "Blob"
    roi_x_percent: float = 35.0
    roi_y_percent: float = 30.0
    roi_w_percent: float = 30.0
    roi_h_percent: float = 40.0
    threshold: float = 0.55
    min_area: float = 80.0
    max_area: float = 20000.0
    blob_polarity: str = "Auto"
    blur_kernel: int = 5
    open_iterations: int = 0
    close_iterations: int = 0
    min_circularity: float = 0.0

    def roi_pixels(self, width: int, height: int) -> tuple[int, int, int, int]:
        x = int(width * self.roi_x_percent / 100.0)
        y = int(height * self.roi_y_percent / 100.0)
        w = int(width * self.roi_w_percent / 100.0)
        h = int(height * self.roi_h_percent / 100.0)
        x = max(0, min(width - 1, x))
        y = max(0, min(height - 1, y))
        w = max(1, min(width - x, w))
        h = max(1, min(height - y, h))
        return x, y, w, h


@dataclass
class CheckerboardConfig:
    columns: int = 7
    rows: int = 6
    square_size_mm: float = 20.0


@dataclass
class CalibrationConfig:
    camera_to_robot: list[float]
    calibrated: bool = False

    @classmethod
    def identity(cls) -> "CalibrationConfig":
        return cls([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], False)


@dataclass
class VisionResult:
    part_ok: bool
    vision_x: float
    vision_y: float
    vision_z: float
    vision_r: float
    score: float = 0.0
    pixel_u: float = 0.0
    pixel_v: float = 0.0
    depth_mm: float = 0.0
    camera_x_mm: float = 0.0
    camera_y_mm: float = 0.0
    camera_z_mm: float = 0.0
    calibrated: bool = False
    message: str = ""

    def to_variables(self) -> dict[str, float | bool | str]:
        return {
            "part_ok": self.part_ok,
            "vision_x": self.vision_x,
            "vision_y": self.vision_y,
            "vision_z": self.vision_z,
            "vision_r": self.vision_r,
            "vision_score": self.score,
            "vision_pixel_u": self.pixel_u,
            "vision_pixel_v": self.pixel_v,
            "vision_depth_mm": self.depth_mm,
            "vision_camera_x_mm": self.camera_x_mm,
            "vision_camera_y_mm": self.camera_y_mm,
            "vision_camera_z_mm": self.camera_z_mm,
            "vision_calibrated": self.calibrated,
            "vision_message": self.message,
        }


def detection_to_variables(result: dict[str, Any] | VisionResult) -> dict[str, float | bool | str]:
    if isinstance(result, VisionResult):
        return result.to_variables()
    return dict(result)
