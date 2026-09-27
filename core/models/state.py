from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any


def now_stamp() -> str:
    return datetime.now().isoformat(timespec="seconds")


@dataclass
class Result:
    success: bool
    message: str = ""
    error_code: str = ""
    timestamp: str = field(default_factory=now_stamp)

    @classmethod
    def ok(cls, message: str = "OK") -> "Result":
        return cls(True, message)

    @classmethod
    def fail(cls, message: str, error_code: str = "ERROR") -> "Result":
        return cls(False, message, error_code)


@dataclass
class Pose:
    x: float = 300.0
    y: float = 0.0
    z: float = 250.0
    r: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Pose":
        return cls(
            float(data.get("x", 300.0)),
            float(data.get("y", 0.0)),
            float(data.get("z", 250.0)),
            float(data.get("r", 0.0)),
        )


@dataclass
class JointState:
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class Position:
    name: str
    pose: Pose
    motion_type: str = "MoveJ"
    speed: float = 20.0
    acceleration: float = 50.0
    approach_offset: float = 0.0
    comment: str = ""

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["pose"] = self.pose.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Position":
        return cls(
            name=str(data["name"]),
            pose=Pose.from_dict(data.get("pose", {})),
            motion_type=str(data.get("motion_type", "MoveJ")),
            speed=float(data.get("speed", 20.0)),
            acceleration=float(data.get("acceleration", 50.0)),
            approach_offset=float(data.get("approach_offset", 0.0)),
            comment=str(data.get("comment", "")),
        )


@dataclass
class SequenceStep:
    no: int
    device: str
    command: str
    target: str = ""
    comment: str = ""
    enabled: bool = True
    condition: str = ""
    speed: float = 20.0
    acceleration: float = 50.0
    status: str = "Ready"
    elapsed_ms: float = 0.0
    vision_profile: str = ""
    calibration_profile: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SequenceStep":
        return cls(
            no=int(data.get("no", 0)),
            device=str(data.get("device", "")),
            command=str(data.get("command", "")),
            target=str(data.get("target", "")),
            comment=str(data.get("comment", "")),
            enabled=bool(data.get("enabled", True)),
            condition=str(data.get("condition", "")),
            speed=float(data.get("speed", 20.0)),
            acceleration=float(data.get("acceleration", 50.0)),
            status=str(data.get("status", "Ready")),
            elapsed_ms=float(data.get("elapsed_ms", 0.0)),
            vision_profile=str(data.get("vision_profile", "")),
            calibration_profile=str(data.get("calibration_profile", "")),
        )


@dataclass
class LogEvent:
    level: str
    source: str
    message: str
    timestamp: str = field(default_factory=now_stamp)

    def to_row(self) -> tuple[str, str, str, str]:
        return self.timestamp, self.level, self.source, self.message
