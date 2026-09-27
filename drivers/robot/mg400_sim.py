from __future__ import annotations

from core.models import JointState, Pose, Result
from drivers.robot.robot_interface import IRobot
from simulation.robot_model import KinematicMG400Model, WorkspaceLimit


class SimulationMG400(IRobot):
    """Deterministic MG400 simulation driver with software-limit validation."""

    def __init__(self) -> None:
        self.connected = False
        self.enabled = False
        self.alarm = ""
        self.model = KinematicMG400Model(WorkspaceLimit())

    def connect(self) -> Result:
        self.connected = True
        return Result.ok("MG400 simulation connected")

    def disconnect(self) -> Result:
        self.connected = False
        self.enabled = False
        return Result.ok("MG400 simulation disconnected")

    def enable(self) -> Result:
        if not self.connected:
            return Result.fail("Robot is not connected", "NOT_CONNECTED")
        self.enabled = True
        return Result.ok("MG400 simulation enabled")

    def disable(self) -> Result:
        self.enabled = False
        return Result.ok("MG400 simulation disabled")

    def clear_error(self) -> Result:
        self.alarm = ""
        return Result.ok("Simulation alarm cleared")

    def get_pose(self) -> Pose:
        return self.model.pose

    def get_joints(self) -> JointState:
        return self.model.joints

    def move_j(self, pose: Pose, speed: float, acceleration: float) -> Result:
        return self._move("MoveJ", pose, speed, acceleration)

    def move_l(self, pose: Pose, speed: float, acceleration: float) -> Result:
        return self._move("MoveL", pose, speed, acceleration)

    def jog(self, axis: str, direction: int, step: float) -> Result:
        if not self.connected:
            return Result.fail("Robot is not connected", "NOT_CONNECTED")
        if not self.enabled:
            return Result.fail("Robot is not enabled", "NOT_ENABLED")
        result = self.model.jog(axis, direction, step)
        if not result.success:
            self.alarm = result.message
        return result

    def stop(self) -> Result:
        return Result.ok("Simulation stop")

    def _move(self, name: str, pose: Pose, speed: float, acceleration: float) -> Result:
        if not self.connected:
            return Result.fail("Robot is not connected", "NOT_CONNECTED")
        if not self.enabled:
            return Result.fail("Robot is not enabled", "NOT_ENABLED")
        result = self.model.move(pose, name, speed, acceleration)
        if not result.success:
            self.alarm = result.message
        return result
