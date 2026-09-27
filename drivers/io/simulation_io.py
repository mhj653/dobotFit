from __future__ import annotations

from core.models import Result


class SimulationIO:
    def __init__(self, channels: int = 8) -> None:
        self.di = {i: False for i in range(1, channels + 1)}
        self.do = {i: False for i in range(1, channels + 1)}
        self.alias = {
            "DO01": "Gripper_Open",
            "DO02": "Gripper_Close",
            "DI01": "Grip_OK",
        }

    def set_do(self, channel: int, state: bool) -> Result:
        if channel not in self.do:
            return Result.fail(f"DO{channel:02d} is not configured", "BAD_DO")
        self.do[channel] = bool(state)
        return Result.ok(f"DO{channel:02d}={'ON' if state else 'OFF'}")

    def set_di(self, channel: int, state: bool) -> Result:
        if channel not in self.di:
            return Result.fail(f"DI{channel:02d} is not configured", "BAD_DI")
        self.di[channel] = bool(state)
        return Result.ok(f"DI{channel:02d}={'ON' if state else 'OFF'}")

    def read_di(self, channel: int) -> bool:
        return bool(self.di.get(channel, False))

