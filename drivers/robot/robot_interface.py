from __future__ import annotations

from abc import ABC, abstractmethod

from core.models import JointState, Pose, Result


class IRobot(ABC):
    @abstractmethod
    def connect(self) -> Result: ...

    @abstractmethod
    def disconnect(self) -> Result: ...

    @abstractmethod
    def enable(self) -> Result: ...

    @abstractmethod
    def disable(self) -> Result: ...

    @abstractmethod
    def clear_error(self) -> Result: ...

    @abstractmethod
    def get_pose(self) -> Pose: ...

    @abstractmethod
    def get_joints(self) -> JointState: ...

    @abstractmethod
    def move_j(self, pose: Pose, speed: float, acceleration: float) -> Result: ...

    @abstractmethod
    def move_l(self, pose: Pose, speed: float, acceleration: float) -> Result: ...

    @abstractmethod
    def jog(self, axis: str, direction: int, step: float) -> Result: ...

    @abstractmethod
    def stop(self) -> Result: ...

