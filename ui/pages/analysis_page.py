from __future__ import annotations

from PySide6.QtWidgets import QHeaderView, QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from core.sequence_engine import SequenceEngine


class AnalysisPage(QWidget):
    def __init__(self, engine: SequenceEngine) -> None:
        super().__init__()
        self.engine = engine
        layout = QVBoxLayout(self)
        title = QLabel("Cycle Analysis")
        title.setProperty("class", "title")
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Step", "Category", "Elapsed s"])
        header = self.table.horizontalHeader()
        header.setStretchLastSection(False)
        for column, width in enumerate([52, 150, 74]):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Fixed)
            self.table.setColumnWidth(column, width)
        layout.addWidget(title)
        layout.addWidget(self.table)
        engine.step_finished.connect(lambda step: self.refresh())

    def refresh(self) -> None:
        self.table.setRowCount(0)
        for no, category, elapsed in self.engine.last_step_times[-200:]:
            row = self.table.rowCount()
            self.table.insertRow(row)
            for col, value in enumerate([no, category, f"{elapsed / 1000.0:.2f}"]):
                self.table.setItem(row, col, QTableWidgetItem(str(value)))
        self.table.scrollToBottom()
