from __future__ import annotations

from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


def card(parent: QWidget | None = None) -> QFrame:
    frame = QFrame(parent)
    frame.setProperty("class", "card")
    return frame


class MetricCard(QFrame):
    def __init__(self, title: str, value: str = "-", subtitle: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setProperty("class", "card")
        layout = QVBoxLayout(self)
        self.title = QLabel(title)
        self.title.setProperty("class", "muted")
        self.value = QLabel(value)
        self.value.setProperty("class", "title")
        self.subtitle = QLabel(subtitle)
        self.subtitle.setProperty("class", "muted")
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.subtitle)
        layout.addStretch()


def labeled_row(label: str, widget: QWidget) -> QWidget:
    row = QWidget()
    layout = QHBoxLayout(row)
    layout.setContentsMargins(0, 2, 0, 2)
    text = QLabel(label)
    text.setMinimumWidth(90)
    layout.addWidget(text)
    layout.addWidget(widget, 1)
    return row

