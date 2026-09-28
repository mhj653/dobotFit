from __future__ import annotations

import unittest

import numpy as np

from core.vision_profiles import default_vision_store, detection_config_from_profile, replace_profile
from drivers.vision.yolo_detector import YoloSegDetector


class YoloVisionTests(unittest.TestCase):
    def test_mask_shape_pose_returns_center_and_angle(self) -> None:
        try:
            import cv2  # type: ignore
        except Exception:
            self.skipTest("OpenCV is not available")
        mask = np.zeros((120, 160), dtype=np.uint8)
        box = cv2.boxPoints(((80, 60), (70, 24), 30))
        cv2.fillPoly(mask, [box.astype(np.int32)], 1)

        detector = YoloSegDetector()
        center, angle, area = detector._shape_pose(mask, (45, 40, 115, 80))

        self.assertAlmostEqual(center[0], 80.0, delta=2.0)
        self.assertAlmostEqual(center[1], 60.0, delta=2.0)
        self.assertAlmostEqual(angle, 30.0, delta=3.0)
        self.assertGreater(area, 1000.0)

    def test_yolo_profile_fields_roundtrip(self) -> None:
        store = default_vision_store()
        payload = {
            "method": "YOLO Segmentation",
            "threshold": 0.66,
            "min_area": 120.0,
            "max_area": 9000.0,
            "yolo_model_path": "models/best.pt",
            "yolo_class_filter": "screw",
            "angle_offset_deg": -12.5,
            "roi": {"x_percent": 10.0, "y_percent": 20.0, "w_percent": 30.0, "h_percent": 40.0},
        }
        store = replace_profile(store, "detection", "YOLO Screw", payload)
        config = detection_config_from_profile(store["profiles"][-1])

        self.assertEqual(config.method, "YOLO Segmentation")
        self.assertEqual(config.yolo_model_path, "models/best.pt")
        self.assertEqual(config.yolo_class_filter, "screw")
        self.assertAlmostEqual(config.angle_offset_deg, -12.5)


if __name__ == "__main__":
    unittest.main()
