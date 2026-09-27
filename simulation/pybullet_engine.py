from __future__ import annotations

import math
from typing import TYPE_CHECKING

from core.models import JointState, Pose

if TYPE_CHECKING:
    from simulation.robot_model import SimulationObject, ToolProfile


class PyBulletEngine:
    """Optional simulation backend wrapper.

    V1 can run without pybullet. When pybullet and an MG400 model are added,
    this class can replace the Qt preview without changing UI command flow.
    """

    def __init__(self) -> None:
        try:
            import pybullet as pybullet  # type: ignore
        except Exception:
            pybullet = None
        self.pybullet = pybullet
        self.client_id: int | None = None
        self.body_ids: list[int] = []
        self.gripper_ids: list[int] = []
        self.vacuum_id: int | None = None
        self.object_body_ids: list[int] = []
        self.camera_view = "Model"
        self.tool_profile: ToolProfile | None = None
        self.ready = False

    @property
    def available(self) -> bool:
        return self.pybullet is not None

    def connect_direct(self) -> bool:
        if self.pybullet is None:
            return False
        if self.client_id is None:
            self.client_id = self.pybullet.connect(self.pybullet.DIRECT)
            self.pybullet.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        return self.client_id >= 0

    def disconnect(self) -> None:
        if self.pybullet is not None and self.client_id is not None:
            self.pybullet.disconnect(self.client_id)
        self.client_id = None
        self.body_ids = []
        self.gripper_ids = []
        self.vacuum_id = None
        self.object_body_ids = []
        self.ready = False

    def initialize_scene(self) -> bool:
        if not self.connect_direct() or self.pybullet is None or self.client_id is None:
            return False
        p = self.pybullet
        p.resetSimulation(physicsClientId=self.client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        plane_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client_id)
        plane_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plane_shape, physicsClientId=self.client_id)
        p.changeDynamics(plane_body, -1, lateralFriction=0.8, physicsClientId=self.client_id)
        self.body_ids = [
            self._box((0.18, 0.18, 0.08), (0.35, 0.35, 0.40, 1.0)),
            self._cylinder(0.07, 0.18, (0.20, 0.25, 0.32, 1.0)),
            self._box((0.28, 0.055, 0.055), (0.82, 0.86, 0.90, 1.0)),
            self._box((0.24, 0.050, 0.050), (0.82, 0.86, 0.90, 1.0)),
            self._box((0.12, 0.045, 0.045), (0.12, 0.16, 0.20, 1.0)),
        ]
        self.gripper_ids = [
            self._box((0.012, 0.012, 0.070), (0.05, 0.08, 0.12, 1.0)),
            self._box((0.012, 0.012, 0.070), (0.05, 0.08, 0.12, 1.0)),
        ]
        self.vacuum_id = self._cylinder(0.026, 0.018, (0.00, 0.72, 0.70, 1.0))
        self.object_body_ids = []
        self.ready = True
        return True

    def configure(self, objects: list["SimulationObject"], tool: "ToolProfile") -> None:
        self.tool_profile = tool
        if not self.ready and not self.initialize_scene():
            return
        if self.pybullet is None or self.client_id is None:
            return
        p = self.pybullet
        for body_id in self.object_body_ids:
            p.removeBody(body_id, physicsClientId=self.client_id)
        self.object_body_ids = [self._scene_body(item) for item in objects]

    def render_camera(self, width: int = 640, height: int = 420) -> tuple[int, int, bytes] | None:
        if not self.ready and not self.initialize_scene():
            return None
        if self.pybullet is None or self.client_id is None:
            return None
        p = self.pybullet
        eye, target, up, fov = self._camera_config()
        view = p.computeViewMatrix(cameraEyePosition=eye, cameraTargetPosition=target, cameraUpVector=up)
        projection = p.computeProjectionMatrixFOV(
            fov=fov,
            aspect=width / max(height, 1),
            nearVal=0.02,
            farVal=3.0,
        )
        image = p.getCameraImage(
            width,
            height,
            viewMatrix=view,
            projectionMatrix=projection,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.client_id,
        )
        rgba = image[2]
        if hasattr(rgba, "tobytes"):
            pixels = rgba.tobytes()
        else:
            pixels = bytes(rgba)
        return width, height, self._darken_render_background(pixels)

    def set_camera_view(self, name: str) -> None:
        self.camera_view = name if name in {"Top", "Front", "Right"} else "Model"

    def sync(
        self,
        pose: Pose,
        joints: JointState,
        gripper_closed: bool,
        objects: list["SimulationObject"] | None = None,
        attached_index: int | None = None,
        tool_profile: "ToolProfile" | None = None,
    ) -> None:
        if not self.ready and not self.initialize_scene():
            return
        if self.pybullet is None or self.client_id is None:
            return
        p = self.pybullet
        tool_profile = tool_profile or self.tool_profile
        points = self._link_points(pose, joints)
        p.resetBasePositionAndOrientation(self.body_ids[0], [0, 0, 0.04], [0, 0, 0, 1], physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(self.body_ids[1], [0, 0, 0.13], [0, 0, 0, 1], physicsClientId=self.client_id)
        for body, start, end in zip(self.body_ids[2:], points[:-1], points[1:]):
            self._place_link(body, start, end)
        tool = points[-1]
        closed_width = (tool_profile.closed_width if tool_profile is not None else 16.0) / 1000.0
        open_width = (tool_profile.open_width if tool_profile is not None else 42.0) / 1000.0
        offset_z = (tool_profile.tcp_offset_z if tool_profile is not None else 65.0) / 1000.0
        tool_name = (tool_profile.name if tool_profile is not None else "Soft Gripper").lower()
        is_vacuum = "vacuum" in tool_name
        spread = closed_width / 2.0 if gripper_closed else open_width / 2.0
        for body, y_offset in zip(self.gripper_ids, [-spread, spread]):
            p.changeVisualShape(
                body,
                -1,
                rgbaColor=[0.02, 0.55, 0.55, 0.22 if is_vacuum else 1.0],
                physicsClientId=self.client_id,
            )
            p.resetBasePositionAndOrientation(
                body,
                [tool[0], tool[1] + y_offset, tool[2] - offset_z * 0.55] if not is_vacuum else [2.0, 2.0, -1.0],
                p.getQuaternionFromEuler([0, 0, 0]),
                physicsClientId=self.client_id,
            )
        if self.vacuum_id is not None:
            p.changeVisualShape(
                self.vacuum_id,
                -1,
                rgbaColor=[0.00, 0.72, 0.70, 1.0 if is_vacuum else 0.05],
                physicsClientId=self.client_id,
            )
            p.resetBasePositionAndOrientation(
                self.vacuum_id,
                [tool[0], tool[1], tool[2] - offset_z] if is_vacuum else [2.0, 2.0, -1.0],
                p.getQuaternionFromEuler([0, math.pi / 2, 0]),
                physicsClientId=self.client_id,
            )
        if objects is not None:
            if len(objects) != len(self.object_body_ids):
                self.configure(objects, tool_profile or self.tool_profile)
            for index, (item, body_id) in enumerate(zip(objects, self.object_body_ids)):
                if attached_index == index:
                    pos = [tool[0], tool[1], tool[2] - offset_z]
                else:
                    pos = [item.x / 1000.0, item.y / 1000.0, item.z / 1000.0]
                p.resetBasePositionAndOrientation(
                    body_id,
                    pos,
                    p.getQuaternionFromEuler([0, 0, 0]),
                    physicsClientId=self.client_id,
                )
        for _ in range(8):
            p.stepSimulation(physicsClientId=self.client_id)

    def object_count(self) -> int:
        return len(self.body_ids) + len(self.gripper_ids) + (1 if self.vacuum_id is not None else 0) + len(self.object_body_ids)

    def _camera_config(self) -> tuple[list[float], list[float], list[float], float]:
        if self.camera_view == "Top":
            return [0.0, 0.0, 1.15], [0.16, 0.0, 0.05], [1.0, 0.0, 0.0], 38
        if self.camera_view == "Front":
            return [0.16, -1.05, 0.32], [0.18, 0.02, 0.15], [0.0, 0.0, 1.0], 34
        if self.camera_view == "Right":
            return [1.05, 0.02, 0.32], [0.18, 0.02, 0.15], [0.0, 0.0, 1.0], 34
        return [0.58, -0.68, 0.76], [0.18, 0.02, 0.10], [0.0, 0.0, 1.0], 38

    def _darken_render_background(self, pixels: bytes) -> bytes:
        data = bytearray(pixels)
        for index in range(0, len(data), 4):
            r, g, b = data[index], data[index + 1], data[index + 2]
            if r > 232 and g > 232 and b > 232:
                data[index] = 15
                data[index + 1] = 23
                data[index + 2] = 42
            elif r > 215 and g > 225 and b > 240:
                data[index] = 40
                data[index + 1] = 74
                data[index + 2] = 132
        return bytes(data)

    def _box(self, size: tuple[float, float, float], rgba: tuple[float, float, float, float]) -> int:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        half = [size[0] / 2, size[1] / 2, size[2] / 2]
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.client_id)
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client_id)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, physicsClientId=self.client_id)

    def _cylinder(self, radius: float, height: float, rgba: tuple[float, float, float, float]) -> int:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=rgba, physicsClientId=self.client_id)
        collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height, physicsClientId=self.client_id)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, physicsClientId=self.client_id)

    def _scene_body(self, item: "SimulationObject") -> int:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        rgba = self._rgba(item.color)
        if item.shape == "Cylinder":
            visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=max(1.0, item.radius) / 1000.0,
                length=max(1.0, item.size_z) / 1000.0,
                rgbaColor=rgba,
                physicsClientId=self.client_id,
            )
            collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=max(1.0, item.radius) / 1000.0,
                height=max(1.0, item.size_z) / 1000.0,
                physicsClientId=self.client_id,
            )
        else:
            half = [max(1.0, item.size_x) / 2000.0, max(1.0, item.size_y) / 2000.0, max(1.0, item.size_z) / 2000.0]
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.client_id)
            collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client_id)
        return p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[item.x / 1000.0, item.y / 1000.0, item.z / 1000.0],
            physicsClientId=self.client_id,
        )

    def _rgba(self, color: str) -> tuple[float, float, float, float]:
        text = color.strip().lstrip("#")
        if len(text) != 6:
            return 0.23, 0.51, 0.96, 1.0
        try:
            return int(text[0:2], 16) / 255.0, int(text[2:4], 16) / 255.0, int(text[4:6], 16) / 255.0, 1.0
        except ValueError:
            return 0.23, 0.51, 0.96, 1.0

    def _place_link(self, body_id: int, start: tuple[float, float, float], end: tuple[float, float, float]) -> None:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        sx, sy, sz = start
        ex, ey, ez = end
        dx, dy, dz = ex - sx, ey - sy, ez - sz
        horizontal = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = -math.atan2(dz, horizontal)
        midpoint = [(sx + ex) / 2, (sy + ey) / 2, (sz + ez) / 2]
        p.resetBasePositionAndOrientation(
            body_id,
            midpoint,
            p.getQuaternionFromEuler([0, pitch, yaw]),
            physicsClientId=self.client_id,
        )

    def _link_points(self, pose: Pose, joints: JointState) -> list[tuple[float, float, float]]:
        yaw = math.radians(joints.j1)
        radius = math.hypot(pose.x, pose.y) / 1000.0
        target = (pose.x / 1000.0, pose.y / 1000.0, pose.z / 1000.0)
        shoulder = (0.0, 0.0, 0.18)
        elbow_r = min(max(radius * 0.55, 0.11), 0.27)
        elbow = (math.cos(yaw) * elbow_r, math.sin(yaw) * elbow_r, 0.29 + joints.j2 / 1200.0)
        wrist_r = min(max(radius * 0.82, 0.18), 0.38)
        wrist = (math.cos(yaw) * wrist_r, math.sin(yaw) * wrist_r, max(0.08, target[2] + 0.055))
        return [shoulder, elbow, wrist, target]
