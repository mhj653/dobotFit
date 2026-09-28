from __future__ import annotations

from typing import Callable

from core.models import Result
from drivers.gripper.gripper_interface import IGripper


OutputWriter = Callable[[int, bool], Result]
InputReader = Callable[[int], tuple[Result, bool]]


class DobotSoftGripper(IGripper):
    def __init__(
        self,
        write_output: OutputWriter | None = None,
        read_input: InputReader | None = None,
        open_output: int = 1,
        close_output: int = 2,
        sensor_input: int = 1,
    ) -> None:
        self.write_output = write_output
        self.read_input = read_input
        self.open_output = open_output
        self.close_output = close_output
        self.sensor_input = sensor_input
        self.connected = write_output is not None
        self.closed = False
        self.last_sensor_state = False

    def configure_io(self, open_output: int, close_output: int, sensor_input: int) -> None:
        self.open_output = open_output
        self.close_output = close_output
        self.sensor_input = sensor_input

    def open(self) -> Result:
        if self.write_output is None:
            return Result.fail("Soft gripper output writer is not configured", "DRIVER_NOT_AVAILABLE")
        close_result = self.write_output(self.close_output, False)
        if not close_result.success:
            return close_result
        open_result = self.write_output(self.open_output, True)
        if not open_result.success:
            return open_result
        self.closed = False
        self.last_sensor_state = False
        return Result.ok(f"Soft gripper open command sent: DO{self.open_output:02d}=ON DO{self.close_output:02d}=OFF")

    def close(self) -> Result:
        if self.write_output is None:
            return Result.fail("Soft gripper output writer is not configured", "DRIVER_NOT_AVAILABLE")
        open_result = self.write_output(self.open_output, False)
        if not open_result.success:
            return open_result
        close_result = self.write_output(self.close_output, True)
        if not close_result.success:
            return close_result
        self.closed = True
        self.last_sensor_state = self._read_sensor()
        suffix = "Grip_OK=ON" if self.last_sensor_state else "Grip_OK=OFF"
        return Result.ok(f"Soft gripper close command sent: DO{self.open_output:02d}=OFF DO{self.close_output:02d}=ON ({suffix})")

    def test(self) -> Result:
        return self.open() if self.closed else self.close()

    def _read_sensor(self) -> bool:
        if self.read_input is None:
            return False
        result, state = self.read_input(self.sensor_input)
        return bool(state) if result.success else False
