# LogView - the app's and the drones' log lines (World.log), newest at the bottom.

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QPlainTextEdit


class LogView(QPlainTextEdit):
    def __init__(self, world):
        super().__init__()
        self.world, self.shown = world, 0
        self.setReadOnly(True)
        self.setMaximumBlockCount(2000)
        self.setFont(QFont("Consolas", 9))

    def refresh(self):
        with self.world.lock:
            new = self.world.logCount - self.shown
            lines = list(self.world.logs)[-new:] if new > 0 else []
            self.shown = self.world.logCount
        for line in lines:
            self.appendPlainText(line)
