from __future__ import annotations

from core.models import Result
from drivers.gripper.gripper_interface import IGripper


class DobotSoftGripper(IGripper):
    def open(self) -> Result:
        return Result.fail("DRIVER NOT AVAILABLE: map gripper I/O from project settings first", "DRIVER_NOT_AVAILABLE")

    def close(self) -> Result:
        return self.open()

    def test(self) -> Result:
        return self.open()

