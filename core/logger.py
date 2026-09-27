from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from core.models import LogEvent


class AppLogger(QObject):
    event_added = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.events: list[LogEvent] = []

    def log(self, level: str, source: str, message: str) -> None:
        event = LogEvent(level=level, source=source, message=message)
        self.events.append(event)
        self.event_added.emit(event)

