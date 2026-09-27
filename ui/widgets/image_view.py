from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy

from drivers.camera.base import CameraFrame


class ImageView(QLabel):
    roi_changed = Signal(tuple)

    def __init__(self, title: str, parent=None) -> None:
        super().__init__(parent)
        self.title = title
        self._image: QImage | None = None
        self._roi_percent: tuple[float, float, float, float] | None = None
        self._marker: tuple[float, float] | None = None
        self._drag_start: tuple[float, float] | None = None
        self._roi_edit_enabled = False
        self.setMouseTracking(True)
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText(title)

    def set_overlay(
        self,
        roi_percent: tuple[float, float, float, float] | None,
        marker: tuple[float, float] | None,
    ) -> None:
        self._roi_percent = roi_percent
        self._marker = marker
        self.update()

    def set_roi_edit_enabled(self, enabled: bool) -> None:
        self._roi_edit_enabled = enabled
        self.setCursor(Qt.CursorShape.CrossCursor if enabled else Qt.CursorShape.ArrowCursor)

    def set_frame(self, frame: CameraFrame | None) -> None:
        if frame is None:
            self._image = None
            self.setText(self.title)
            self.update()
            return
        if frame.pixel_format != "rgb8" or frame.channels != 3:
            self._image = None
            self.setText(f"Unsupported frame: {frame.pixel_format}")
            return
        self._image = QImage(frame.data, frame.width, frame.height, frame.width * 3, QImage.Format.Format_RGB888).copy()
        self._update_pixmap()

    def resizeEvent(self, event) -> None:  # noqa: N802
        self._update_pixmap()
        super().resizeEvent(event)

    def paintEvent(self, event) -> None:  # noqa: N802
        if self._image is not None:
            super().paintEvent(event)
            self._paint_overlay()
            return
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#d7dde5"))
        painter.setPen(QColor("#6b7280"))
        for x in range(20, self.width(), 44):
            painter.drawLine(x, 0, x, self.height())
        for y in range(20, self.height(), 44):
            painter.drawLine(0, y, self.width(), y)
        painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.title)

    def _update_pixmap(self) -> None:
        if self._image is None or self.width() <= 0 or self.height() <= 0:
            return
        pixmap = QPixmap.fromImage(self._image)
        self.setPixmap(pixmap.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))

    def _paint_overlay(self) -> None:
        if self._image is None:
            return
        scale = min(self.width() / self._image.width(), self.height() / self._image.height())
        image_w = self._image.width() * scale
        image_h = self._image.height() * scale
        offset_x = (self.width() - image_w) / 2.0
        offset_y = (self.height() - image_h) / 2.0
        painter = QPainter(self)
        if self._roi_percent is not None:
            x, y, w, h = self._roi_percent
            rect_x = offset_x + image_w * x / 100.0
            rect_y = offset_y + image_h * y / 100.0
            rect_w = image_w * w / 100.0
            rect_h = image_h * h / 100.0
            painter.setPen(QPen(QColor("#22c55e"), 2))
            painter.drawRect(int(rect_x), int(rect_y), int(rect_w), int(rect_h))
        if self._marker is not None:
            marker_x = offset_x + self._marker[0] * scale
            marker_y = offset_y + self._marker[1] * scale
            painter.setPen(QPen(QColor("#ef4444"), 2))
            painter.drawLine(int(marker_x - 8), int(marker_y), int(marker_x + 8), int(marker_y))
            painter.drawLine(int(marker_x), int(marker_y - 8), int(marker_x), int(marker_y + 8))

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if self._roi_edit_enabled and event.button() == Qt.MouseButton.LeftButton and self._image is not None:
            self._drag_start = self._event_to_percent(event)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._roi_edit_enabled and self._drag_start is not None and self._image is not None:
            current = self._event_to_percent(event)
            if current is not None:
                self._roi_percent = self._normalized_roi(self._drag_start, current)
                self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if self._roi_edit_enabled and event.button() == Qt.MouseButton.LeftButton and self._drag_start is not None:
            current = self._event_to_percent(event)
            start = self._drag_start
            self._drag_start = None
            if current is not None:
                roi = self._normalized_roi(start, current)
                if roi[2] >= 1.0 and roi[3] >= 1.0:
                    self._roi_percent = roi
                    self.roi_changed.emit(roi)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _event_to_percent(self, event) -> tuple[float, float] | None:
        if self._image is None:
            return None
        scale = min(self.width() / self._image.width(), self.height() / self._image.height())
        image_w = self._image.width() * scale
        image_h = self._image.height() * scale
        offset_x = (self.width() - image_w) / 2.0
        offset_y = (self.height() - image_h) / 2.0
        point = event.position()
        x = (point.x() - offset_x) / image_w * 100.0
        y = (point.y() - offset_y) / image_h * 100.0
        return max(0.0, min(100.0, x)), max(0.0, min(100.0, y))

    def _normalized_roi(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> tuple[float, float, float, float]:
        x0, x1 = sorted([start[0], end[0]])
        y0, y1 = sorted([start[1], end[1]])
        return x0, y0, max(1.0, x1 - x0), max(1.0, y1 - y0)
