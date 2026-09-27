from __future__ import annotations

import math
from dataclasses import dataclass

from core.models import JointState, Pose


@dataclass(frozen=True)
class MG400KinematicSpec:
    """Simulation geometry for a DOBOT MG400-style 4-axis desktop robot.

    The public MG400 specification defines a 440 mm working radius and 190 mm
    square base. This model uses two equal planar links to reproduce that reach
    envelope while keeping the simulation deterministic and lightweight.
    """

    link1_mm: float = 220.0
    link2_mm: float = 220.0
    base_size_mm: float = 190.0
    base_height_mm: float = 70.0
    column_height_mm: float = 230.0
    arm_clearance_mm: float = 70.0
    j1_min_deg: float = -160.0
    j1_max_deg: float = 160.0
    j2_min_deg: float = 0.0
    j2_max_deg: float = 170.0
    j4_min_deg: float = -360.0
    j4_max_deg: float = 360.0

    @property
    def max_reach_mm(self) -> float:
        return self.link1_mm + self.link2_mm


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MG400Kinematics:
    def __init__(self, spec: MG400KinematicSpec | None = None) -> None:
        self.spec = spec or MG400KinematicSpec()

    def inverse(self, pose: Pose) -> JointState | None:
        x = pose.x
        y = pose.y
        radius_sq = x * x + y * y
        l1 = self.spec.link1_mm
        l2 = self.spec.link2_mm
        denominator = 2.0 * l1 * l2
        if denominator == 0:
            return None

        cos_elbow = (radius_sq - l1 * l1 - l2 * l2) / denominator
        if cos_elbow < -1.0 or cos_elbow > 1.0:
            return None

        elbow = math.acos(clamp(cos_elbow, -1.0, 1.0))
        shoulder = math.atan2(y, x) - math.atan2(l2 * math.sin(elbow), l1 + l2 * math.cos(elbow))
        j1 = self._normalize_deg(math.degrees(shoulder))
        j2 = math.degrees(elbow)
        j4 = self._normalize_deg(pose.r - j1 - j2)

        if not (self.spec.j1_min_deg <= j1 <= self.spec.j1_max_deg):
            return None
        if not (self.spec.j2_min_deg <= j2 <= self.spec.j2_max_deg):
            return None
        if not (self.spec.j4_min_deg <= j4 <= self.spec.j4_max_deg):
            return None
        return JointState(j1=j1, j2=j2, j3=pose.z, j4=j4)

    def forward(self, joints: JointState) -> Pose:
        theta1 = math.radians(joints.j1)
        theta12 = math.radians(joints.j1 + joints.j2)
        x = self.spec.link1_mm * math.cos(theta1) + self.spec.link2_mm * math.cos(theta12)
        y = self.spec.link1_mm * math.sin(theta1) + self.spec.link2_mm * math.sin(theta12)
        r = self._normalize_deg(joints.j1 + joints.j2 + joints.j4)
        return Pose(x=x, y=y, z=joints.j3, r=r)

    def link_points_m(self, pose: Pose, joints: JointState) -> list[tuple[float, float, float]]:
        theta1 = math.radians(joints.j1)
        theta12 = math.radians(joints.j1 + joints.j2)
        l1 = self.spec.link1_mm / 1000.0
        l2 = self.spec.link2_mm / 1000.0
        tcp_z = pose.z / 1000.0
        arm_z = max(tcp_z + self.spec.arm_clearance_mm / 1000.0, self.spec.column_height_mm / 1000.0)

        shoulder = (0.0, 0.0, arm_z)
        elbow = (l1 * math.cos(theta1), l1 * math.sin(theta1), arm_z)
        wrist = (
            elbow[0] + l2 * math.cos(theta12),
            elbow[1] + l2 * math.sin(theta12),
            arm_z,
        )
        tcp = (pose.x / 1000.0, pose.y / 1000.0, tcp_z)
        return [shoulder, elbow, wrist, tcp]

    def fallback_joints(self, pose: Pose) -> JointState:
        radius = math.hypot(pose.x, pose.y)
        return JointState(
            j1=clamp(math.degrees(math.atan2(pose.y, pose.x)), self.spec.j1_min_deg, self.spec.j1_max_deg),
            j2=clamp((radius / max(self.spec.max_reach_mm, 1.0)) * 140.0, self.spec.j2_min_deg, self.spec.j2_max_deg),
            j3=pose.z,
            j4=clamp(pose.r, self.spec.j4_min_deg, self.spec.j4_max_deg),
        )

    def _normalize_deg(self, value: float) -> float:
        while value > 180.0:
            value -= 360.0
        while value < -180.0:
            value += 360.0
        return value
