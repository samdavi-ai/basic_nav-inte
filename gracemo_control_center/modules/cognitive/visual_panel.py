"""Local optical trajectory and diagnostics from the kernel's current perception reply."""
import json
import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPen
from PySide6.QtWidgets import QGraphicsScene, QGraphicsView, QLabel, QTextEdit, QVBoxLayout, QWidget


class VisualPanel(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.status = QLabel(self)
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)
        self.details = QTextEdit(self)
        self.details.setReadOnly(True)
        for widget in (self.status, self.view, self.details):
            layout.addWidget(widget)

    def toPlainText(self):
        return self.details.toPlainText()

    def setPlainText(self, text):
        self.details.setPlainText(text)
        self.scene.clear()
        try:
            data = json.loads(text)
        except ValueError:
            self.status.setText(text)
            return
        if not isinstance(data, dict) or not data.get("current"):
            self.status.setText("Current perception unavailable")
            return
        visual = (data.get("sample") or {}).get("visual_odometry") or {}
        self.status.setText(f"Camera trajectory · optical X/Z · {visual.get('state', 'unavailable')}")
        previous = None
        for row in visual.get("trajectory", [])[-128:]:
            if not isinstance(row,list) or len(row) != 5:
                previous = None
                continue
            frame, state, x, _, z = row
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (frame,x,z)):
                previous = None
                continue
            x, y = 100*x, -100*z
            if previous and state != "relocalized" and 0 < frame - previous[0] <= 5:
                self.scene.addLine(previous[1], previous[2], x, y, QPen(QColor("#4889b9")))
            color = QColor("#ef943c" if state == "relocalized" else "#4889b9")
            self.scene.addEllipse(x-1,y-1,2,2,QPen(color),color)
            previous = frame, x, y
        if previous:
            bounds = self.scene.itemsBoundingRect().adjusted(-5,-5,5,5)
            self.view.setSceneRect(bounds)
            self.view.fitInView(bounds, Qt.AspectRatioMode.KeepAspectRatio)
