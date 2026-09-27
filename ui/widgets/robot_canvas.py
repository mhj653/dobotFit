from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from core.models import Pose


class RobotCanvas(QWidget):
    target_pose_changed = Signal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.pose = Pose()
        self.path: list[Pose] = []
        self.gripper_closed = False
        self.backend = "Kinematic"
        self.physics_objects = 0
        self.render_width = 0
        self.render_height = 0
        self.render_rgba: bytes | None = None
        self.render_scale = 1.0
        self.render_fill = False
        self.interactive_enabled = True
        self.view_name = "Model"
        self.yaw = math.radians(-38)
        self.pitch = math.radians(26)
        self.zoom = 1.0
        self._drag_start: QPointF | None = None
        self._drag_pose: Pose | None = None
        self.setMinimumHeight(360)
        self.setMouseTracking(True)

    def set_state(
        self,
        pose: Pose,
        path: list[Pose] | None = None,
        gripper_closed: bool = False,
        backend: str = "Kinematic",
        physics_objects: int = 0,
        render_width: int = 0,
        render_height: int = 0,
        render_rgba: bytes | None = None,
    ) -> None:
        self.pose = pose
        self.path = path or []
        self.gripper_closed = gripper_closed
        self.backend = backend
        self.physics_objects = physics_objects
        self.render_width = render_width
        self.render_height = render_height
        self.render_rgba = render_rgba
        self.update()

    def set_pose(self, pose: Pose) -> None:
        self.set_state(pose, self.path, self.gripper_closed)

    def set_render_scale(self, scale: float, fill: bool = False) -> None:
        self.render_scale = max(0.5, min(2.0, scale))
        self.render_fill = fill
        self.update()

    def set_interactive_enabled(self, enabled: bool) -> None:
        self.interactive_enabled = enabled
        self.setCursor(Qt.CursorShape.ArrowCursor if not enabled else Qt.CursorShape.OpenHandCursor)

    def set_view(self, name: str) -> None:
        self.view_name = name
        if name == "Top":
            self.yaw = 0.0
            self.pitch = math.radians(90)
        elif name == "Front":
            self.yaw = 0.0
            self.pitch = 0.0
        elif name == "Right":
            self.yaw = math.radians(-90)
            self.pitch = 0.0
        elif name in {"Home", "Fit View", "Model"}:
            self.yaw = math.radians(-38)
            self.pitch = math.radians(26)
            self.zoom = 1.0
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self._draw_pybullet_frame(painter):
            return
        painter.fillRect(self.rect(), QColor("#eaf1f8"))
        self._draw_floor(painter)
        self._draw_workspace_3d(painter)
        self._draw_path_3d(painter)
        self._draw_robot(painter)
        painter.setPen(QColor("#46566b"))
        painter.drawText(18, 24, f"MG400 Simulation View - {self.view_name}")
        painter.drawText(18, 44, f"Backend: {self.backend}  Physics objects: {self.physics_objects}")
        painter.drawText(18, 64, "Drag to rotate, wheel to zoom")

    def _draw_pybullet_frame(self, painter: QPainter) -> bool:
        if not self._has_pybullet_frame():
            return False
        image = QImage(self.render_rgba, self.render_width, self.render_height, QImage.Format.Format_RGBA8888).copy()
        painter.fillRect(self.rect(), QColor("#111827"))
        if self.render_fill:
            scaled = image.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatioByExpanding, Qt.TransformationMode.SmoothTransformation)
            source_x = (scaled.width() - self.width()) / 2
            source_y = (scaled.height() - self.height()) / 2
            painter.drawImage(QRectF(self.rect()), scaled, QRectF(source_x, source_y, self.width(), self.height()))
        else:
            fit_size = self.size()
            fit_size.setWidth(max(1, int(self.width() * self.render_scale)))
            fit_size.setHeight(max(1, int(self.height() * self.render_scale)))
            scaled = image.scaled(fit_size, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            target_x = (self.width() - scaled.width()) / 2
            target_y = (self.height() - scaled.height()) / 2
            painter.drawImage(QRectF(target_x, target_y, scaled.width(), scaled.height()), scaled, QRectF(scaled.rect()))

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(15, 23, 42, 215))
        painter.drawRoundedRect(QRectF(12, 12, 294, 66), 6, 6)
        painter.setFont(QFont("Arial", 10))
        painter.setPen(QColor("#f8fafc"))
        painter.drawText(24, 32, "PYBULLET RENDER ACTIVE")
        painter.setPen(QColor("#93c5fd"))
        painter.drawText(24, 52, f"DIRECT physics world  |  Objects: {self.physics_objects}")
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(24, 72, f"TCP: X {self.pose.x:.1f}  Y {self.pose.y:.1f}  Z {self.pose.z:.1f}  R {self.pose.r:.1f}")
        painter.setFont(QFont("Arial", 9))
        painter.setPen(QColor("#fde68a"))
        if self.interactive_enabled:
            painter.drawText(18, self.height() - 16, "Left drag: X/Y move  |  Wheel: Z move")
        else:
            painter.drawText(18, self.height() - 16, "Preview only")
        return True

    def _draw_robot(self, painter: QPainter) -> None:
        points = self._robot_points()
        self._draw_block(painter, (-58, -58, 0), (58, 58, 72), QColor("#f8fafc"), QColor("#334155"))
        self._draw_cylinder_top(painter, (0, 0, 82), 48, QColor("#334155"))
        self._draw_link(painter, points[0], points[1], 34, QColor("#111827"), QColor("#e5e7eb"))
        self._draw_link(painter, points[1], points[2], 32, QColor("#111827"), QColor("#f8fafc"))
        self._draw_link(painter, points[2], points[3], 26, QColor("#111827"), QColor("#e5e7eb"))
        self._draw_link(painter, points[3], points[4], 18, QColor("#111827"), QColor("#475569"))
        for point in points[1:4]:
            self._draw_joint(painter, point)
        self._draw_gripper_3d(painter, points[-1])
        painter.setPen(QColor("#334155"))
        painter.drawText(self._project((0, -80, 55)), "MG400")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if not self.interactive_enabled:
            return
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = event.position()
            self._drag_pose = Pose(self.pose.x, self.pose.y, self.pose.z, self.pose.r) if self._has_pybullet_frame() else None

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if not self.interactive_enabled:
            return
        if self._drag_start is None:
            return
        if self._drag_pose is not None and self._has_pybullet_frame():
            delta = event.position() - self._drag_start
            pose = Pose(
                self._drag_pose.x + delta.x() * 0.85 - delta.y() * 0.35,
                self._drag_pose.y + delta.x() * 0.55 + delta.y() * 0.65,
                self._drag_pose.z,
                self._drag_pose.r,
            )
            pose = self._clamp_pose(pose)
            self.pose = pose
            self.target_pose_changed.emit(pose)
            return
        delta = event.position() - self._drag_start
        self._drag_start = event.position()
        self.yaw += delta.x() * 0.01
        self.pitch = max(math.radians(-10), min(math.radians(80), self.pitch + delta.y() * 0.006))
        self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        self._drag_start = None
        self._drag_pose = None

    def wheelEvent(self, event) -> None:  # noqa: N802
        if not self.interactive_enabled:
            return
        if self._has_pybullet_frame():
            step = 8.0 if event.angleDelta().y() > 0 else -8.0
            pose = self._clamp_pose(Pose(self.pose.x, self.pose.y, self.pose.z + step, self.pose.r))
            self.pose = pose
            self.target_pose_changed.emit(pose)
            return
        self.zoom = max(0.55, min(1.8, self.zoom * (1.08 if event.angleDelta().y() > 0 else 0.92)))
        self.update()

    def _has_pybullet_frame(self) -> bool:
        return self.backend == "PyBullet" and bool(self.render_rgba) and self.render_width > 0 and self.render_height > 0

    def _clamp_pose(self, pose: Pose) -> Pose:
        radius = math.hypot(pose.x, pose.y)
        if radius > 440.0:
            scale = 440.0 / radius
            pose.x *= scale
            pose.y *= scale
        elif radius < 80.0:
            scale = 80.0 / max(radius, 1.0)
            pose.x *= scale
            pose.y *= scale
        pose.x = max(-440.0, min(440.0, pose.x))
        pose.y = max(-440.0, min(440.0, pose.y))
        pose.z = max(-5.0, min(400.0, pose.z))
        return pose

    def _draw_floor(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor("#cad6e4"), 1))
        for value in range(-480, 481, 80):
            painter.drawLine(self._project((-480, value, 0)), self._project((480, value, 0)))
            painter.drawLine(self._project((value, -480, 0)), self._project((value, 480, 0)))
        painter.setPen(QPen(QColor("#ef4444"), 3))
        painter.drawLine(self._project((0, 0, 0)), self._project((180, 0, 0)))
        painter.drawText(self._project((205, 0, 0)), "X")
        painter.setPen(QPen(QColor("#16a34a"), 3))
        painter.drawLine(self._project((0, 0, 0)), self._project((0, 180, 0)))
        painter.drawText(self._project((0, 205, 0)), "Y")
        painter.setPen(QPen(QColor("#2563eb"), 3))
        painter.drawLine(self._project((0, 0, 0)), self._project((0, 0, 180)))
        painter.drawText(self._project((0, 0, 205)), "Z")

    def _draw_workspace_3d(self, painter: QPainter) -> None:
        outer = QPolygonF([self._project((math.cos(t) * 440, math.sin(t) * 440, 2)) for t in self._circle_steps()])
        inner = QPolygonF([self._project((math.cos(t) * 80, math.sin(t) * 80, 3)) for t in self._circle_steps()])
        painter.setBrush(QColor(37, 99, 235, 32))
        painter.setPen(QPen(QColor("#60a5fa"), 2, Qt.PenStyle.DashLine))
        painter.drawPolygon(outer)
        painter.setBrush(QColor(239, 68, 68, 28))
        painter.setPen(QPen(QColor("#f87171"), 1, Qt.PenStyle.DashLine))
        painter.drawPolygon(inner)

    def _draw_path_3d(self, painter: QPainter) -> None:
        if len(self.path) < 2:
            return
        path = QPainterPath()
        path.moveTo(self._project((self.path[0].x, self.path[0].y, self.path[0].z)))
        for pose in self.path[1:]:
            path.lineTo(self._project((pose.x, pose.y, pose.z)))
        painter.setPen(QPen(QColor("#f59e0b"), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawPath(path)

    def _robot_points(self) -> list[tuple[float, float, float]]:
        radius = math.hypot(self.pose.x, self.pose.y)
        angle = math.atan2(self.pose.y, self.pose.x)
        shoulder = (0.0, 0.0, 110.0)
        elbow_r = max(110.0, min(280.0, radius * 0.50))
        elbow = (math.cos(angle) * elbow_r, math.sin(angle) * elbow_r, 235.0 + (self.pose.z - 200.0) * 0.18)
        wrist_r = max(170.0, min(385.0, radius * 0.82))
        wrist = (math.cos(angle) * wrist_r, math.sin(angle) * wrist_r, max(80.0, self.pose.z + 60.0))
        tool = (self.pose.x, self.pose.y, self.pose.z)
        tcp = (self.pose.x, self.pose.y, self.pose.z - 65.0)
        return [(0.0, 0.0, 50.0), shoulder, elbow, wrist, tool, tcp]

    def _draw_link(
        self,
        painter: QPainter,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        width: int,
        edge: QColor,
        fill: QColor,
    ) -> None:
        p1 = self._project(start)
        p2 = self._project(end)
        painter.setPen(QPen(edge, width + 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(p1, p2)
        painter.setPen(QPen(fill, width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(p1, p2)

    def _draw_joint(self, painter: QPainter, point: tuple[float, float, float]) -> None:
        p = self._project(point)
        painter.setPen(QPen(QColor("#111827"), 3))
        painter.setBrush(QColor("#334155"))
        painter.drawEllipse(p, 19 * self.zoom, 19 * self.zoom)
        painter.setBrush(QColor("#e5e7eb"))
        painter.drawEllipse(p, 10 * self.zoom, 10 * self.zoom)

    def _draw_gripper_3d(self, painter: QPainter, tcp: tuple[float, float, float]) -> None:
        spread = 16 if self.gripper_closed else 42
        for side in [-1, 1]:
            start = (tcp[0], tcp[1] + side * spread * 0.45, tcp[2])
            end = (tcp[0], tcp[1] + side * spread, tcp[2] - 62)
            self._draw_link(painter, start, end, 9, QColor("#111827"), QColor("#2563eb") if self.gripper_closed else QColor("#cbd5e1"))
        painter.setPen(QColor("#334155"))
        painter.drawText(self._project((tcp[0] + 20, tcp[1] + 20, tcp[2] - 70)), "Closed" if self.gripper_closed else "Open")

    def _draw_block(
        self,
        painter: QPainter,
        minimum: tuple[float, float, float],
        maximum: tuple[float, float, float],
        fill: QColor,
        edge: QColor,
    ) -> None:
        x1, y1, z1 = minimum
        x2, y2, z2 = maximum
        faces = [
            [(x1, y1, z2), (x2, y1, z2), (x2, y2, z2), (x1, y2, z2)],
            [(x1, y2, z1), (x2, y2, z1), (x2, y2, z2), (x1, y2, z2)],
            [(x2, y1, z1), (x2, y2, z1), (x2, y2, z2), (x2, y1, z2)],
        ]
        colors = [fill, QColor("#e2e8f0"), QColor("#cbd5e1")]
        painter.setPen(QPen(edge, 1))
        for face, color in zip(faces, colors):
            painter.setBrush(color)
            painter.drawPolygon(QPolygonF([self._project(point) for point in face]))

    def _draw_cylinder_top(self, painter: QPainter, center: tuple[float, float, float], radius: float, color: QColor) -> None:
        polygon = QPolygonF([self._project((center[0] + math.cos(t) * radius, center[1] + math.sin(t) * radius, center[2])) for t in self._circle_steps(32)])
        painter.setBrush(color)
        painter.setPen(QPen(QColor("#111827"), 2))
        painter.drawPolygon(polygon)

    def _project(self, point: tuple[float, float, float]) -> QPointF:
        x, y, z = point
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        xr = x * cy - y * sy
        yr = x * sy + y * cy
        zr = z
        ys = yr * cp - zr * sp
        scale = min(self.width(), self.height()) / 850.0 * self.zoom
        return QPointF(self.width() * 0.52 + xr * scale, self.height() * 0.66 + ys * scale)

    def _circle_steps(self, count: int = 72):
        for index in range(count):
            yield math.tau * index / count
