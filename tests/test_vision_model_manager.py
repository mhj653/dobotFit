from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from core.vision_model_manager import import_yolo_model


class VisionModelManagerTests(unittest.TestCase):
    def test_import_yolo_model_copies_pt_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = root / "trained.pt"
            target_dir = root / "models"
            source.write_bytes(b"model")
            with patch("core.vision_model_manager.models_dir", return_value=target_dir):
                imported = import_yolo_model(source)

            self.assertEqual(imported, target_dir / "trained.pt")
            self.assertEqual(imported.read_bytes(), b"model")

    def test_import_yolo_model_rejects_unknown_suffix(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "model.txt"
            source.write_text("nope", encoding="utf-8")
            with self.assertRaises(ValueError):
                import_yolo_model(source)

    def test_import_yolo_model_requires_overwrite_for_existing_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = root / "trained.onnx"
            target_dir = root / "models"
            target_dir.mkdir()
            source.write_bytes(b"new")
            (target_dir / source.name).write_bytes(b"old")
            with patch("core.vision_model_manager.models_dir", return_value=target_dir):
                with self.assertRaises(FileExistsError):
                    import_yolo_model(source)
                imported = import_yolo_model(source, overwrite=True)

            self.assertEqual(imported.read_bytes(), b"new")


if __name__ == "__main__":
    unittest.main()
