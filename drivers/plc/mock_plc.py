from __future__ import annotations

from core.models import Result


class MockPLC:
    def __init__(self) -> None:
        self.connected = True
        self.outputs: dict[str, bool] = {}

    def write_output(self, name: str, state: bool) -> Result:
        self.outputs[name] = bool(state)
        return Result.ok(f"PLC {name}={'ON' if state else 'OFF'}")

