from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass
class GripperIOConfig:
    open_output: int = 1
    close_output: int = 2
    sensor_input: int = 1

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "GripperIOConfig":
        data = data or {}
        return cls(
            open_output=_channel(data.get("open_output", data.get("open_do", 1)), 1),
            close_output=_channel(data.get("close_output", data.get("close_do", 2)), 2),
            sensor_input=_channel(data.get("sensor_input", data.get("grip_di", 1)), 1),
        )

    def to_dict(self) -> dict[str, int]:
        return asdict(self)

    @property
    def open_label(self) -> str:
        return f"DO{self.open_output:02d}"

    @property
    def close_label(self) -> str:
        return f"DO{self.close_output:02d}"

    @property
    def sensor_label(self) -> str:
        return f"DI{self.sensor_input:02d}"


def _channel(value: Any, default: int) -> int:
    try:
        text = str(value).upper().replace("DO", "").replace("DI", "").strip()
        channel = int(text)
    except (TypeError, ValueError):
        channel = default
    return max(1, min(64, channel))
