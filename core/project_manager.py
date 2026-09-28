from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from core.models import Pose, Position, SequenceStep
from core.tool_config import GripperIOConfig
from core.vision_profiles import (
    default_calibration_store,
    default_vision_store,
    normalize_calibration_store,
    normalize_vision_store,
)


DEFAULT_POSITIONS = [
    Position("HOME", Pose(300, 0, 250, 0), comment="Default home"),
    Position("PICK_APPROACH", Pose(350, 100, 180, 90), "MoveJ", comment="Pick approach"),
    Position("PICK", Pose(350, 100, 120, 90), "MoveL", comment="Pick position"),
    Position("CAMERA", Pose(320, 50, 200, 90), "MoveJ", comment="Camera position"),
    Position("PLACE", Pose(250, -80, 150, -90), "MoveJ", comment="Place position"),
    Position("INSPECTION", Pose(280, 120, 180, 0), "MoveJ", comment="Vision position"),
]

DEFAULT_SEQUENCE = [
    SequenceStep(1, "Robot", "MoveJ", "HOME"),
    SequenceStep(2, "Robot", "MoveJ", "PICK_APPROACH"),
    SequenceStep(3, "Robot", "MoveL", "PICK"),
    SequenceStep(4, "Gripper", "Close"),
    SequenceStep(5, "Wait", "Time", "0.30"),
    SequenceStep(6, "Wait", "DI", "1"),
    SequenceStep(7, "Robot", "MoveL", "PICK_APPROACH"),
    SequenceStep(8, "Vision", "Detect", "ScrewHole"),
    SequenceStep(9, "Robot", "MoveL", "VISION_XYZ"),
    SequenceStep(10, "Gripper", "Open"),
    SequenceStep(11, "PLC", "SetOutput", "COMPLETE=ON"),
]

DEFAULT_ROBOT = {
    "model": "MG400",
    "mode": "Simulation",
    "ip": "192.168.1.6",
    "dashboard_port": 29999,
    "move_port": 30003,
    "feedback_port": 30004,
    "timeout_s": 1.5,
}

DEFAULT_TOOLS = {"soft_gripper": GripperIOConfig().to_dict()}


class ProjectManager:
    def __init__(self, project_path: Path) -> None:
        self.project_path = project_path
        self.project_path.mkdir(parents=True, exist_ok=True)

    def load(self) -> dict[str, Any]:
        positions_path = self.project_path / "positions.json"
        sequence_path = self.project_path / "sequence.json"
        io_path = self.project_path / "io_mapping.json"
        project = self._read_json(self.project_path / "project.json", {"name": self.project_path.name})
        robot = {**DEFAULT_ROBOT, **self._read_json(self.project_path / "robot.json", DEFAULT_ROBOT)}
        positions = [Position.from_dict(item) for item in self._read_json(positions_path, [p.to_dict() for p in DEFAULT_POSITIONS])]
        sequence = [SequenceStep.from_dict(item) for item in self._read_json(sequence_path, [s.to_dict() for s in DEFAULT_SEQUENCE])]
        io_mapping = self._read_json(io_path, {"DO01": "Gripper_Open", "DO02": "Gripper_Close", "DI01": "Grip_OK"})
        tools = self._read_json(self.project_path / "tools.json", DEFAULT_TOOLS)
        vision = normalize_vision_store(self._read_json(self.project_path / "vision.json", default_vision_store()))
        calibration = normalize_calibration_store(self._read_json(self.project_path / "calibration.json", default_calibration_store()))
        return {
            "project": project,
            "robot": robot,
            "positions": positions,
            "sequence": sequence,
            "io_mapping": io_mapping,
            "tools": tools,
            "vision": vision,
            "calibration": calibration,
        }

    def save(
        self,
        positions: list[Position],
        sequence: list[SequenceStep],
        io_mapping: dict[str, str],
        robot_config: dict[str, Any] | None = None,
        tools: dict[str, Any] | None = None,
        vision: dict[str, Any] | None = None,
        calibration: dict[str, Any] | None = None,
    ) -> None:
        self.project_path.mkdir(parents=True, exist_ok=True)
        self._write_json(self.project_path / "project.json", {"name": self.project_path.name, "version": "0.1.0"})
        self._write_json(self.project_path / "positions.json", [p.to_dict() for p in positions])
        self._write_json(self.project_path / "sequence.json", [s.to_dict() for s in sequence])
        self._write_json(self.project_path / "io_mapping.json", io_mapping)
        self._write_json(
            self.project_path / "robot.json",
            robot_config
            or DEFAULT_ROBOT,
        )
        self._write_json(self.project_path / "tools.json", tools or DEFAULT_TOOLS)
        self._write_json(self.project_path / "vision.json", normalize_vision_store(vision))
        self._write_json(self.project_path / "calibration.json", normalize_calibration_store(calibration))

    def _read_json(self, path: Path, default: Any) -> Any:
        if not path.exists():
            return default
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def _write_json(self, path: Path, data: Any) -> None:
        with path.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, ensure_ascii=False)
