from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

from core.models import JointState, Pose
from simulation.mg400_kinematics import MG400Kinematics, MG400KinematicSpec

if TYPE_CHECKING:
    from simulation.robot_model import SimulationObject, ToolProfile


class PyBulletEngine:
    """Optional PyBullet renderer for the MG400 kinematic simulation."""

    def __init__(self) -> None:
        try:
            import pybullet as pybullet  # type: ignore
        except Exception:
            pybullet = None
        self.pybullet = pybullet
        self.client_id: int | None = None
        self.robot_id: int | None = None
        self.robot_joint_indices: dict[str, int] = {}
        self.body_ids: list[int] = []
        self.gripper_ids: list[int] = []
        self.vacuum_id: int | None = None
        self.object_body_ids: list[int] = []
        self.camera_view = "Model"
        self.tool_profile: ToolProfile | None = None
        self.spec = MG400KinematicSpec()
        self.kinematics = MG400Kinematics(self.spec)
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
        self.robot_id = None
        self.robot_joint_indices = {}
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
        self.robot_id = self._load_mg400_urdf()
        if self.robot_id is None:
            self.body_ids = [
                self._box((0.19, 0.19, 0.07), (0.32, 0.33, 0.38, 1.0)),
                self._cylinder(0.065, 0.23, (0.18, 0.22, 0.28, 1.0)),
                self._box((0.22, 0.058, 0.052), (0.82, 0.86, 0.88, 1.0)),
                self._box((0.22, 0.052, 0.048), (0.82, 0.86, 0.88, 1.0)),
                self._box((0.085, 0.040, 0.040), (0.08, 0.12, 0.18, 1.0)),
            ]
        else:
            self.body_ids = []
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
        if self.robot_id is not None:
            self._sync_urdf_robot(pose)
        elif self.body_ids:
            p.resetBasePositionAndOrientation(self.body_ids[0], [0, 0, 0.04], [0, 0, 0, 1], physicsClientId=self.client_id)
            p.resetBasePositionAndOrientation(self.body_ids[1], [0, 0, 0.13], [0, 0, 0, 1], physicsClientId=self.client_id)
            for body, start, end in zip(self.body_ids[2:], points[:-1], points[1:]):
                self._place_link(body, start, end)
        tool, tool_orientation = self._tool_frame(points[-1], pose.r)
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
                self._tool_local_position(tool, tool_orientation, [0.0, y_offset, -offset_z * 0.55])
                if not is_vacuum
                else [2.0, 2.0, -1.0],
                tool_orientation,
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
                self._tool_local_position(tool, tool_orientation, [0.0, 0.0, -offset_z]) if is_vacuum else [2.0, 2.0, -1.0],
                tool_orientation,
                physicsClientId=self.client_id,
            )
        if objects is not None:
            if len(objects) != len(self.object_body_ids):
                self.configure(objects, tool_profile or self.tool_profile)
            for index, (item, body_id) in enumerate(zip(objects, self.object_body_ids)):
                if attached_index == index:
                    pos = self._tool_local_position(tool, tool_orientation, [0.0, 0.0, -offset_z])
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
        robot_count = 1 if self.robot_id is not None else len(self.body_ids)
        return robot_count + len(self.gripper_ids) + (1 if self.vacuum_id is not None else 0) + len(self.object_body_ids)

    @property
    def uses_urdf_model(self) -> bool:
        return self.robot_id is not None

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

    def _load_mg400_urdf(self) -> int | None:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        source = self._mg400_urdf_path()
        if source is None:
            return None
        patched = self._patched_urdf_path(source)
        try:
            robot_id = p.loadURDF(
                str(patched),
                basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True,
                flags=p.URDF_USE_INERTIA_FROM_FILE,
                physicsClientId=self.client_id,
            )
        except Exception:
            return None
        self.robot_joint_indices = {}
        for index in range(p.getNumJoints(robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(robot_id, index, physicsClientId=self.client_id)
            self.robot_joint_indices[info[1].decode("utf-8")] = index
        return robot_id

    def _mg400_urdf_path(self) -> Path | None:
        base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[1]))
        candidates = [
            base / "assets" / "mg400_description" / "urdf" / "mg400_description.urdf",
            Path(__file__).resolve().parents[1] / "assets" / "mg400_description" / "urdf" / "mg400_description.urdf",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    def _patched_urdf_path(self, source: Path) -> Path:
        asset_root = source.parents[1]
        cache_dir = Path(tempfile.gettempdir()) / "robot_automation_studio"
        cache_dir.mkdir(parents=True, exist_ok=True)
        target = cache_dir / "mg400_description_pybullet.urdf"
        text = source.read_text(encoding="utf-8")
        text = text.replace("package://mg400_description/", asset_root.as_posix() + "/")
        target.write_text(text, encoding="utf-8")
        return target

    def _sync_urdf_robot(self, pose: Pose) -> None:
        p = self.pybullet
        assert p is not None and self.client_id is not None and self.robot_id is not None
        targets = self._urdf_joint_targets(pose)
        for name, value in targets.items():
            index = self.robot_joint_indices.get(name)
            if index is not None:
                p.resetJointState(self.robot_id, index, value, physicsClientId=self.client_id)

    def _tool_frame(self, fallback_position: tuple[float, float, float], fallback_r_deg: float) -> tuple[list[float], list[float]]:
        p = self.pybullet
        assert p is not None and self.client_id is not None
        if self.robot_id is not None:
            flange_index = self.robot_joint_indices.get("j4")
            if flange_index is not None:
                link_state = p.getLinkState(self.robot_id, flange_index, computeForwardKinematics=True, physicsClientId=self.client_id)
                return list(link_state[4]), list(link_state[5])
        return list(fallback_position), list(p.getQuaternionFromEuler([0, 0, math.radians(fallback_r_deg)]))

    def _tool_local_position(self, position: list[float], orientation: list[float], local_offset: list[float]) -> list[float]:
        p = self.pybullet
        assert p is not None
        world_position, _world_orientation = p.multiplyTransforms(position, orientation, local_offset, [0, 0, 0, 1])
        return list(world_position)

    def _urdf_joint_targets(self, pose: Pose) -> dict[str, float]:
        radius = math.hypot(pose.x, pose.y)
        yaw = math.atan2(pose.y, pose.x)
        reach_t = self._clamp((radius - 80.0) / 360.0, 0.0, 1.0)
        height_t = self._clamp((pose.z - 5.0) / 395.0, 0.0, 1.0)

        shoulder = self._clamp(1.18 - reach_t * 0.92 + (height_t - 0.55) * 0.28, -0.14, 1.39)
        elbow = self._clamp(0.18 + reach_t * 0.82 + (0.50 - height_t) * 0.18, 0.0, 1.39)
        wrist_pitch = self._clamp(elbow, -math.pi, math.pi)
        wrist_roll = self._clamp(math.radians(pose.r), -math.pi, math.pi)

        return {
            "j1": yaw,
            "j2": shoulder,
            "j2_2": shoulder,
            "j3": elbow,
            "j3_1": -shoulder,
            "j3_2": -shoulder,
            "j4_1": -wrist_pitch,
            "j4_2": wrist_pitch,
            "j4": wrist_roll,
        }

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
        return self.kinematics.link_points_m(pose, joints)

    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
