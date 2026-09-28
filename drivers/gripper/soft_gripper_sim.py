from __future__ import annotations

from core.models import Result
from drivers.gripper.gripper_interface import IGripper


class SimulationSoftGripper(IGripper):
    def __init__(self) -> None:
        self.connected = True
        self.closed = False
        self.fingers = "4-Finger"
        self.open_output = 1
        self.close_output = 2
        self.sensor_input = 1

    def configure_io(self, open_output: int, close_output: int, sensor_input: int) -> None:
        self.open_output = open_output
        self.close_output = close_output
        self.sensor_input = sensor_input

    def open(self) -> Result:
        self.closed = False
        return Result.ok("Soft gripper opened")

    def close(self) -> Result:
        self.closed = True
        return Result.ok("Soft gripper closed")

    def test(self) -> Result:
        self.closed = not self.closed
        return Result.ok("Soft gripper test toggled")
