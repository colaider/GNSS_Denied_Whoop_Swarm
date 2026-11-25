# MapView - top view of the start frame: x forward = up, y left = left, metres, grid every 10 cm.
# Per drone (own colour): trail, position + heading, setpoint (+), goal (x), the trajectory sent
# (dashed) and, for the selected one, the box it may fly in (hover point +- maxDistM).
# Left click: clicked(x, y) in the start frame (the main window sends the selected drone there).

import math

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal

from planner.trajectory import MAX_DIST

COLOURS = ["#1d4ed8", "#dc2626", "#059669", "#d97706", "#7c3aed", "#db2777"]


def colour(i):
    return COLOURS[i % len(COLOURS)]


class MapView(pg.PlotWidget):
    clicked = Signal(float, float)      # start frame x, y

    def __init__(self):
        super().__init__(background="w")
        self.setAspectLocked(True)
        self.invertX(True)              # plot x = y (left is +), plot y = x (up is forward)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.getAxis("bottom").setTickSpacing(0.5, 0.1)
        self.getAxis("left").setTickSpacing(0.5, 0.1)
        self.setLabel("bottom", "y  (m, left +)")
        self.setLabel("left", "x  (m, forward +)")
        self.setXRange(-1.0, 1.0)
        self.setYRange(-1.0, 1.0)
        self.setMenuEnabled(False)
        self.hideButtons()
        self.addItem(pg.ScatterPlotItem([0], [0], symbol="s", size=8, brush=None, pen=pg.mkPen("#999999")))   # start point
        self.box = pg.PlotDataItem(pen=pg.mkPen("#bbbbbb", width=1, style=Qt.DashLine))
        self.addItem(self.box)
        self.items = {}                 # drone index -> its plot items
        self.scene().sigMouseClicked.connect(self.onClick)

    def onClick(self, ev):
        if ev.button() != Qt.LeftButton or not self.plotItem.vb.sceneBoundingRect().contains(ev.scenePos()):
            return
        p = self.plotItem.vb.mapSceneToView(ev.scenePos())
        self.clicked.emit(p.y(), p.x())

    def droneItems(self, d):
        if d.index not in self.items:
            c = colour(d.index)
            it = {"trail": pg.PlotDataItem(pen=pg.mkPen(c, width=1)),
                  "path": pg.PlotDataItem(pen=pg.mkPen(c, width=2, style=Qt.DashLine)),
                  "heading": pg.PlotDataItem(pen=pg.mkPen(c, width=3)),
                  "pos": pg.ScatterPlotItem(size=12, brush=pg.mkBrush(c), pen=None),
                  "setpoint": pg.ScatterPlotItem(size=16, symbol="+", brush=pg.mkBrush("#ef4444"), pen=pg.mkPen("#ef4444")),
                  "goal": pg.ScatterPlotItem(size=14, symbol="x", brush=pg.mkBrush(c), pen=pg.mkPen(c)),
                  "label": pg.TextItem(d.name, color=c, anchor=(0.5, 1.6))}
            for item in it.values():
                self.addItem(item)
            self.items[d.index] = it
        return self.items[d.index]

    def refresh(self, world, selected):
        with world.lock:
            for d in world.drones:
                it = self.droneItems(d)
                if d.trail:
                    xs, ys, _ = zip(*d.trail)
                    it["trail"].setData(ys, xs)
                it["path"].setData([p[1] for p in d.path], [p[0] for p in d.path])
                it["goal"].setData([d.goal[1]], [d.goal[0]]) if d.goal else it["goal"].clear()
                if d.telem:
                    r = d.telem
                    it["pos"].setData([r["y"]], [r["x"]])
                    hx, hy = r["x"] + 0.08 * math.cos(r["yaw"]), r["y"] + 0.08 * math.sin(r["yaw"])
                    it["heading"].setData([r["y"], hy], [r["x"], hx])
                    it["setpoint"].setData([r["ty"]], [r["tx"]])
                    it["label"].setPos(r["y"], r["x"])
                    it["label"].setText(f"{d.name}  {r['z'] * 100:.0f} cm")
            s = selected
            if s is not None and s.origin is not None:
                ox, oy, _ = s.origin
                xs = [ox - MAX_DIST, ox - MAX_DIST, ox + MAX_DIST, ox + MAX_DIST, ox - MAX_DIST]
                ys = [oy - MAX_DIST, oy + MAX_DIST, oy + MAX_DIST, oy - MAX_DIST, oy - MAX_DIST]
                self.box.setData(ys, xs)
            else:
                self.box.clear()
