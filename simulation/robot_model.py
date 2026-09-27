from __future__ import annotations

import math
from dataclasses import dataclass, field

from core.models import JointState, Pose, Result
from simulation.mg400_kinematics import MG400Kinematics, MG400KinematicSpec
from simulation.pybullet_engine import PyBulletEngine


@dataclass
class WorkspaceLimit:
    min_radius: float = 80.0
    max_radius: float = 440.0
    min_x: float = -440.0
    max_x: float = 440.0
    min_y: float = -440.0
    max_y: float = 440.0
    min_z: float = -5.0
    max_z: float = 400.0
    floor_z: float = -5.0


@dataclass
class SimulationObject:
    name: str
    shape: str = "Box"
    x: float = 260.0
    y: float = 80.0
    z: float = 25.0
    size_x: float = 60.0
    size_y: float = 60.0
    size_z: float = 50.0
    radius: float = 30.0
    color: str = "#3b82f6"
    attached: bool = False


@dataclass
class ToolProfile:
    name: str = "Soft Gripper"
    tcp_offset_z: float = 65.0
    open_width: float = 42.0
    closed_width: float = 16.0
    grip_range: float = 38.0


@dataclass
class SimulationSnapshot:
    pose: Pose
    joints: JointState
    path: list[Pose]
    gripper_closed: bool = False
    backend: str = "Kinematic"
    physics_objects: int = 0
    render_width: int = 0
    render_height: int = 0
    render_rgba: bytes | None = None
    objects: list[SimulationObject] = field(default_factory=list)
    tool: ToolProfile = field(default_factory=ToolProfile)
    warnings: list[str] = field(default_factory=list)


class KinematicMG400Model:
    """Hardware-free MG400 simulation model with SCARA-style kinematics."""

    def __init__(self, limits: WorkspaceLimit | None = None) -> None:
        self.limits = limits or WorkspaceLimit()
        self.spec = MG400KinematicSpec()
        self.kinematics = MG400Kinematics(self.spec)
        self.pose = Pose()
        self.joints = self.estimate_joints(self.pose)
        self.last_path: list[Pose] = [self.pose]
        self.gripper_closed = False
        self.objects = self._default_objects()
        self.tool = ToolProfile()
        self.attached_object_index: int | None = None
        self.warnings: list[str] = []
        self.pybullet = PyBulletEngine()
        self.backend = "PyBullet" if self.pybullet.initialize_scene() else "Kinematic"
        self.pybullet.configure(self.objects, self.tool)
        self._sync_backend()

    def move(self, target: Pose, motion: str, speed: float, acceleration: float) -> Result:
        validation = self.validate(target)
        if not validation.success:
            return validation
        self.last_path = self._build_path(self.pose, target, motion, speed, acceleration)
        self.warnings = self.check_path(self.last_path)
        self.pose = target
        self.joints = self.estimate_joints(target)
        self._sync_backend()
        message = f"{motion} simulation complete with {len(self.last_path)} path samples"
        if self.warnings:
            message += f" ({len(self.warnings)} simulation warning(s))"
        return Result.ok(message)

    def jog(self, axis: str, direction: int, step: float) -> Result:
        target = Pose(self.pose.x, self.pose.y, self.pose.z, self.pose.r)
        delta = step * (1 if direction >= 0 else -1)
        if axis == "X":
            target.x += delta
        elif axis == "Y":
            target.y += delta
        elif axis == "Z":
            target.z += delta
        elif axis == "R":
            target.r += delta
        else:
            return Result.fail(f"Unsupported jog axis {axis}", "BAD_AXIS")
        return self.move(target, "Jog", 10.0, 30.0)

    def set_gripper(self, closed: bool) -> None:
        self.gripper_closed = closed
        if closed:
            self._attach_nearest_object()
        else:
            self._release_object()
        self._sync_backend()

    def set_tool_profile(self, tool: ToolProfile) -> None:
        self.tool = tool
        if self.backend == "PyBullet":
            self.pybullet.configure(self.objects, self.tool)
        self._sync_backend()

    def set_scene_objects(self, objects: list[SimulationObject]) -> None:
        self.objects = objects
        self.attached_object_index = None
        for item in self.objects:
            item.attached = False
        if self.backend == "PyBullet":
            self.pybullet.configure(self.objects, self.tool)
        self._sync_backend()

    def reset_scene_objects(self) -> None:
        self.set_scene_objects(self._default_objects())

    def set_view(self, name: str) -> None:
        if self.backend == "PyBullet":
            self.pybullet.set_camera_view(name)

    def snapshot(self, render: bool = True) -> SimulationSnapshot:
        frame = self.pybullet.render_camera() if render and self.backend == "PyBullet" else None
        return SimulationSnapshot(
            self.pose,
            self.joints,
            list(self.last_path),
            self.gripper_closed,
            self.backend,
            self.pybullet.object_count() if self.backend == "PyBullet" else 0,
            frame[0] if frame else 0,
            frame[1] if frame else 0,
            frame[2] if frame else None,
            [SimulationObject(**vars(item)) for item in self.objects],
            ToolProfile(**vars(self.tool)),
            list(self.warnings),
        )

    def validate(self, pose: Pose) -> Result:
        limits = self.limits
        if not (limits.min_x <= pose.x <= limits.max_x):
            return Result.fail(f"X is outside software limit ({limits.min_x:.0f}..{limits.max_x:.0f} mm)", "LIMIT_X")
        if not (limits.min_y <= pose.y <= limits.max_y):
            return Result.fail(f"Y is outside software limit ({limits.min_y:.0f}..{limits.max_y:.0f} mm)", "LIMIT_Y")
        if not (limits.min_z <= pose.z <= limits.max_z):
            return Result.fail(f"Z is outside software limit ({limits.min_z:.0f}..{limits.max_z:.0f} mm)", "LIMIT_Z")
        if pose.z < limits.floor_z:
            return Result.fail("Target violates configured floor check", "FLOOR")
        radius = math.hypot(pose.x, pose.y)
        if radius > limits.max_radius:
            return Result.fail(f"Target exceeds maximum reach radius {limits.max_radius:.0f} mm", "REACH")
        if radius < limits.min_radius:
            return Result.fail("Target is too close to robot base", "REACH_MIN")
        if self.kinematics.inverse(pose) is None:
            return Result.fail("Target has no valid MG400 joint solution", "KINEMATICS")
        return Result.ok()

    def check_path(self, path: list[Pose]) -> list[str]:
        warnings: list[str] = []
        for pose in path:
            validation = self.validate(pose)
            if not validation.success:
                warnings.append(validation.message)
                break
        for item in self.objects:
            if item.attached:
                continue
            if self._path_intersects_object(path, item):
                warnings.append(f"Path approaches object '{item.name}'")
        return warnings

    def estimate_joints(self, pose: Pose) -> JointState:
        return self.kinematics.inverse(pose) or self.kinematics.fallback_joints(pose)

    def forward_kinematics(self, joints: JointState) -> Pose:
        return self.kinematics.forward(joints)

    def _build_path(self, start: Pose, target: Pose, motion: str, speed: float, acceleration: float) -> list[Pose]:
        distance = math.sqrt((target.x - start.x) ** 2 + (target.y - start.y) ** 2 + (target.z - start.z) ** 2)
        samples = max(8, min(80, int(distance / max(speed, 1.0) * 4 + acceleration / 15)))
        path = []
        for index in range(samples + 1):
            t = index / samples
            if motion == "MoveJ":
                t = 0.5 - 0.5 * math.cos(math.pi * t)
            path.append(
                Pose(
                    x=start.x + (target.x - start.x) * t,
                    y=start.y + (target.y - start.y) * t,
                    z=start.z + (target.z - start.z) * t,
                    r=start.r + (target.r - start.r) * t,
                )
            )
        return path

    def _sync_backend(self) -> None:
        if self.backend == "PyBullet":
            self.pybullet.sync(self.pose, self.joints, self.gripper_closed, self.objects, self.attached_object_index, self.tool)

    def _tool_point(self) -> tuple[float, float, float]:
        return self.pose.x, self.pose.y, self.pose.z - self.tool.tcp_offset_z

    def _attach_nearest_object(self) -> None:
        if self.attached_object_index is not None:
            return
        tx, ty, tz = self._tool_point()
        nearest_index: int | None = None
        nearest_distance = self.tool.grip_range
        for index, item in enumerate(self.objects):
            dx = item.x - tx
            dy = item.y - ty
            dz = item.z - tz
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= nearest_distance:
                nearest_distance = distance
                nearest_index = index
        if nearest_index is not None:
            self.attached_object_index = nearest_index
            self.objects[nearest_index].attached = True

    def _release_object(self) -> None:
        if self.attached_object_index is None:
            return
        tx, ty, tz = self._tool_point()
        item = self.objects[self.attached_object_index]
        item.x = tx
        item.y = ty
        item.z = max(item.size_z / 2.0, tz)
        item.attached = False
        self.attached_object_index = None

    def _path_intersects_object(self, path: list[Pose], item: SimulationObject) -> bool:
        margin = 18.0
        for pose in path:
            if abs(pose.x - item.x) <= item.size_x / 2.0 + margin and abs(pose.y - item.y) <= item.size_y / 2.0 + margin:
                if abs((pose.z - self.tool.tcp_offset_z) - item.z) <= item.size_z / 2.0 + margin:
                    return True
        return False

    def _default_objects(self) -> list[SimulationObject]:
        return [
            SimulationObject("BOX_1", "Box", 320.0, 90.0, 25.0, 60.0, 60.0, 50.0, 30.0, "#3b82f6"),
            SimulationObject("PART_1", "Cylinder", 230.0, -90.0, 20.0, 40.0, 40.0, 40.0, 24.0, "#22c55e"),
        ]
