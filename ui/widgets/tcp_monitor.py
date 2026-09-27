from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
)


class TcpMonitorWidget(QGroupBox):
    def __init__(self, title: str = "Robot TCP Monitor", max_rows: int = 500) -> None:
        super().__init__(title)
        self.max_rows = max_rows
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 10, 8, 8)
        layout.setSpacing(6)

        toolbar = QHBoxLayout()
        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        clear_button = QPushButton("Clear")
        self.errors_only = QCheckBox("Errors only")
        clear_button.clicked.connect(self.clear)
        toolbar.addWidget(self.pause_button)
        toolbar.addWidget(clear_button)
        toolbar.addWidget(self.errors_only)
        toolbar.addStretch()

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Time", "Dir", "Port", "ms", "Message"])
        header = self.table.horizontalHeader()
        header.setStretchLastSection(False)
        for column, width in enumerate([82, 42, 54, 58, 260]):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.table.setColumnWidth(column, width)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setWordWrap(False)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)

        layout.addLayout(toolbar)
        layout.addWidget(self.table, 1)

    def append_packet(self, packet: object) -> None:
        if self.pause_button.isChecked() or not isinstance(packet, dict):
            return
        direction = str(packet.get("direction", ""))
        is_error = bool(packet.get("error", False)) or direction == "ERR"
        if self.errors_only.isChecked() and not is_error:
            return
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [
            str(packet.get("time", "")),
            direction,
            str(packet.get("port", "")),
            f"{float(packet.get('elapsed_ms', 0.0)):.1f}",
            str(packet.get("message", "")),
        ]
        for column, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setToolTip(value)
            if is_error:
                item.setForeground(QBrush(QColor("#ef4444")))
            elif direction == "TX":
                item.setForeground(QBrush(QColor("#38bdf8")))
            elif direction == "RX":
                item.setForeground(QBrush(QColor("#22c55e")))
            self.table.setItem(row, column, item)
        while self.table.rowCount() > self.max_rows:
            self.table.removeRow(0)
        self.table.scrollToBottom()

    def clear(self) -> None:
        self.table.setRowCount(0)
