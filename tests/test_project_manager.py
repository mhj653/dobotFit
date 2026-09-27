from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from core.project_manager import DEFAULT_POSITIONS, DEFAULT_SEQUENCE, ProjectManager


class ProjectManagerTests(unittest.TestCase):
    def test_load_defaults_and_save_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            manager = ProjectManager(Path(temp) / "Demo")
            data = manager.load()
            self.assertEqual(len(data["positions"]), len(DEFAULT_POSITIONS))
            self.assertEqual(len(data["sequence"]), len(DEFAULT_SEQUENCE))

            manager.save(data["positions"], data["sequence"], data["io_mapping"])
            loaded = manager.load()
            self.assertEqual(loaded["positions"][0].name, "HOME")
            self.assertEqual(loaded["sequence"][0].command, "MoveJ")


if __name__ == "__main__":
    unittest.main()

