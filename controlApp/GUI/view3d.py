# View3D - the start frame in 3D: floor grid (10 cm, the +-maxDistM box), height axis, and per drone
# (own colour) its trail, position with a drop line to the floor, setpoint (+), goal, the trajectory
# sent and the tracked camera points at their measured height (PointDepth: blue = floor .. red = BOX_M
# up; filled = sure). Drag to turn, wheel to zoom.

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt

from GUI.mapView import colour
from GUI.orbit3d import Orbit3D, gridLines
from planner.trajectory import MAX_DIST, MAX_HEIGHT

DEPTH_SURE, DEPTH_SHOW = 0.03, 0.10     # m: depth std below = sure (filled), above DEPTH_SHOW not drawn
BOX_M = 0.20                            # point colour: height 0 blue .. this red
TOP = 1.0                               # height axis drawn to here (m)


def depthColour(z):
    k = min(max(z / BOX_M, 0.0), 1.0)
    return (int(59 + k * 161), int(130 - k * 92), int(246 - k * 208))


class View3D(Orbit3D):
    def __init__(self):
        super().__init__(radius=1.6)
        self.grid = self.line("#e6e6e6")
        self.axes = self.line("#b0b0b0")
        self.posts = self.line("#eeeeee")
        self.ticks = []
        for k in range(1, int(TOP / 0.2 + 0.5) + 1):
            t = pg.TextItem(f"{k * 20} cm", color="#999999", anchor=(0, 0.5))
            self.addItem(t)
            self.ticks.append((t, k * 0.2))
        self.names = [(pg.TextItem("x", color="#888888"), (MAX_DIST + 0.1, 0, 0)),
                      (pg.TextItem("y", color="#888888"), (0, MAX_DIST + 0.1, 0))]
        for t, _ in self.names:
            self.addItem(t)
        self.info = pg.TextItem("", color="#333333", anchor=(0, 0))
        self.addItem(self.info)
        self.items = {}

    def droneItems(self, d):
        if d.index not in self.items:
            c = colour(d.index)
            it = {"trail": self.line(c), "path": self.line(c, 2, Qt.DashLine), "drop": self.line("#94a3b8", 1, Qt.DotLine),
                  "setpoint": self.line("#ef4444", 2), "heading": self.line(c, 3),
                  "floor": pg.ScatterPlotItem(size=6, brush=pg.mkBrush("#94a3b8"), pen=None),
                  "pos": pg.ScatterPlotItem(size=12, brush=pg.mkBrush(c), pen=None),
                  "goal": pg.ScatterPlotItem(size=14, symbol="x", brush=pg.mkBrush(c), pen=pg.mkPen(c)),
                  "depth": pg.ScatterPlotItem(size=7)}
            for k in ("floor", "pos", "goal", "depth"):
                self.addItem(it[k])
            self.items[d.index] = it
        return self.items[d.index]

    def placeStatic(self):
        self.moved = False
        self.setLine(self.grid, gridLines(MAX_DIST, 0.1), pairs=True)
        self.setLine(self.axes, np.array([(-MAX_DIST, 0, 0), (MAX_DIST, 0, 0), (0, -MAX_DIST, 0), (0, MAX_DIST, 0),
                                          (0, 0, 0), (0, 0, TOP)]), pairs=True)
        corners = [(sx * MAX_DIST, sy * MAX_DIST) for sx in (-1, 1) for sy in (-1, 1)]
        self.setLine(self.posts, np.array([p for x, y in corners for p in ((x, y, 0), (x, y, TOP))]), pairs=True)
        for t, z in self.ticks:
            u, v = self.xy((0, 0, z))
            t.setPos(u[0], v[0])
        for t, p in self.names:
            u, v = self.xy(p)
            t.setPos(u[0], v[0])

    def refresh(self, world, selected):
        if self.moved:
            self.placeStatic()
        lines = []
        with world.lock:
            for d in world.drones:
                it = self.droneItems(d)
                self.setLine(it["trail"], np.array(d.trail) if d.trail else None)
                self.setLine(it["path"], np.array(d.path) if d.path else None)
                if d.goal:
                    u, v = self.xy(d.goal)
                    it["goal"].setData(u, v)
                else:
                    it["goal"].clear()
                if d.telem:
                    r = d.telem
                    p = np.array([r["x"], r["y"], r["z"]])
                    u, v = self.xy(p)
                    it["pos"].setData(u, v)
                    u, v = self.xy((r["x"], r["y"], 0))
                    it["floor"].setData(u, v)
                    self.setLine(it["drop"], np.array([p, (r["x"], r["y"], 0)]))
                    h = p + 0.1 * np.array([np.cos(r["yaw"]), np.sin(r["yaw"]), 0])
                    self.setLine(it["heading"], np.array([p, h]))
                    t, e = np.array([r["tx"], r["ty"], r["tz"]]), 0.04
                    self.setLine(it["setpoint"], np.array([t - (e, 0, 0), t + (e, 0, 0), t - (0, e, 0), t + (0, e, 0),
                                                           t - (0, 0, e), t + (0, 0, e)]), pairs=True)
                pts = [q for q in d.depth if q[4] <= DEPTH_SHOW]
                if pts:
                    u, v = self.xy(np.array([q[1:4] for q in pts]))
                    it["depth"].setData(u, v, brush=[pg.mkBrush(*depthColour(q[3])) if q[4] < DEPTH_SURE else None
                                                     for q in pts],
                                        pen=[pg.mkPen(depthColour(q[3])) for q in pts])
                else:
                    it["depth"].clear()
                if d is selected and d.depth:
                    sure = sorted(q[3] for q in d.depth if q[4] < DEPTH_SURE)
                    lines.append(f"{d.name}: points 3D {len(d.depth)} (sure {len(sure)})" +
                                 (f"  height median {sure[len(sure) // 2] * 100:.0f} cm, max {sure[-1] * 100:.0f} cm"
                                  if sure else ""))
        (x0, _), (_, y1) = self.plotItem.vb.viewRange()
        self.info.setPos(x0, y1)
        self.info.setText("\n".join(lines) or "3D: drag to turn, wheel to zoom")
