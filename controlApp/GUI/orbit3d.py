# Orbit3D - a 3D view drawn on a 2D pyqtgraph plot (no OpenGL needed): drag = turn around z / tilt,
# wheel = zoom. Subclasses build their items once and call self.xy(points) to place them.
#   line(pen)  -> a PlotDataItem;  self.setLine(item, points, pairs=False)  (pairs: separate segments)

import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt

from utilities.geometry import project


class Orbit3D(pg.PlotWidget):
    def __init__(self, radius=1.0, az=225.0, el=25.0):
        super().__init__(background="w")
        self.az, self.el, self.radius = math.radians(az), math.radians(el), radius
        self.setAspectLocked(True)
        self.hideAxis("left")
        self.hideAxis("bottom")
        self.setMenuEnabled(False)
        self.hideButtons()
        self.plotItem.vb.setMouseEnabled(False, False)
        self.mouse = None
        self.moved = True               # the view turned: static items need placing again
        self.fit()

    def fit(self):
        self.setXRange(-self.radius, self.radius, padding=0)
        self.setYRange(-self.radius, self.radius, padding=0)

    def xy(self, points):
        return project(points, self.az, self.el)

    def line(self, colour, width=1, style=Qt.SolidLine):
        item = pg.PlotDataItem(pen=pg.mkPen(colour, width=width, style=style))
        self.addItem(item)
        return item

    def setLine(self, item, points, pairs=False):
        if points is None or len(points) == 0:
            item.clear()
            return
        u, v = self.xy(points)
        item.setData(u, v, connect="pairs" if pairs else "all")

    def mousePressEvent(self, ev):
        self.mouse = ev.position()

    def mouseMoveEvent(self, ev):       # left / right: turn around z, up / down: tilt
        if self.mouse is None:
            return
        p = ev.position()
        self.az -= (p.x() - self.mouse.x()) * 0.01
        self.el = min(max(self.el + (p.y() - self.mouse.y()) * 0.01, -1.5), 1.5)
        self.mouse, self.moved = p, True

    def mouseReleaseEvent(self, ev):
        self.mouse = None

    def wheelEvent(self, ev):
        self.radius *= 0.9 if ev.angleDelta().y() > 0 else 1.1
        self.fit()


def gridLines(half, step, z=0.0):       # floor grid as segment pairs: (2 * lines * 2, 3)
    ks = np.arange(-half, half + step / 2, step)
    seg = []
    for k in ks:
        seg += [(k, -half, z), (k, half, z), (-half, k, z), (half, k, z)]
    return np.array(seg)
