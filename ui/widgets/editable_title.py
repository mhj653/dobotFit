from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QLabel


class EditableTitleLabel(QLabel):
    double_clicked = Signal()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        self.double_clicked.emit()
        super().mouseDoubleClickEvent(event)
