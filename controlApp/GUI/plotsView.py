# PlotsView - the selected drone's last WINDOW_S seconds, live (time 0 = now), x axes linked.
# Rows: height, x / y, speed, roll / pitch (sent vs IMU: does Betaflight follow?), throttle (vs the hover
# throttle the drone expects from the battery), battery, ToF, camera tracking. Dashed = what was asked.
# New rows: add to ROWS: (title, [(field or computed name, scale, colour, dashed, legend), ...]).

import time

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QScrollArea

from communication.protocol import MSG, DRONES

WINDOW_S = 30.0
ROW_PX = 130
FIELDS = MSG.structs["HoverTelem"].fields
HOVER_V = DRONES.defaults["hoverThrottleV"]   # hover throttle = this / battery volts (the default drone, drones.h)
BLUE, RED, GREEN, PURPLE, GREY, ORANGE = "#1d4ed8", "#dc2626", "#059669", "#7c3aed", "#94a3b8", "#d97706"

ROWS = [
    ("height (cm)", [("z", 100, BLUE, False, "z"), ("tz", 100, RED, True, "target"), ("tofHeight", 100, GREY, False, "ToF")]),
    ("x / y (cm)", [("x", 100, BLUE, False, "x"), ("tx", 100, BLUE, True, "x target"),
                    ("y", 100, GREEN, False, "y"), ("ty", 100, GREEN, True, "y target")]),
    ("speed (cm/s)", [("vx", 100, BLUE, False, "vx"), ("vy", 100, GREEN, False, "vy"), ("vz", 100, RED, False, "vz")]),
    ("roll / pitch (deg)", [("roll", 1, RED, True, "roll sent"), ("attRoll", 1, RED, False, "roll IMU"),
                            ("pitch", 1, PURPLE, True, "pitch sent"), ("attPitch", 1, PURPLE, False, "pitch IMU")]),
    ("throttle", [("throttle", 1, BLUE, False, "throttle"), ("hover", 1, GREY, True, "hover (battery)")]),
    ("battery (V)", [("battery", 1, ORANGE, False, "battery")]),
    ("ToF", [("tofSignal", 1, BLUE, False, "signal MCPS"), ("tofValid", 10, RED, False, "valid x10"),
             ("tofStatus", 1, GREY, False, "status")]),
    ("camera", [("tracked", 1, GREY, False, "tracked"), ("inliers", 1, GREEN, False, "inliers"),
                ("rms", 10, RED, False, "rms px x10")]),
]
FLOW_COLS = {"tracked": 1, "inliers": 2, "rms": 3}     # in Drone.flowHistory rows


class PlotsView(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self.glw.setMinimumHeight(ROW_PX * len(ROWS))
        self.setWidget(self.glw)
        self.curves, first = [], None
        for i, (title, series) in enumerate(ROWS):
            p = self.glw.addPlot(row=i, col=0)
            p.setLabel("left", title)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setMenuEnabled(False)
            p.hideButtons()
            p.addLegend(offset=(5, 2), labelTextSize="8pt", colCount=len(series))
            if first is None:
                first = p
            else:
                p.setXLink(first)
            if i < len(ROWS) - 1:
                p.getAxis("bottom").setStyle(showValues=False)
            for key, scale, c, dashed, name in series:
                pen = pg.mkPen(c, width=1.5, style=Qt.DashLine if dashed else Qt.SolidLine)
                self.curves.append((key, scale, p.plot(pen=pen, name=name)))
        first.setXRange(-WINDOW_S, 0, padding=0)
        self.glw.getItem(len(ROWS) - 1, 0).setLabel("bottom", "s (0 = now)")

    def refresh(self, world, drone):
        if drone is None:
            return
        with world.lock:
            hist = list(drone.history)
            flow = list(drone.flowHistory)
        now = time.time()
        t = np.array([h[0] for h in hist]) - now
        v = np.array([h[1] for h in hist], float).reshape(-1, len(FIELDS))
        ft = np.array([f[0] for f in flow]) - now
        fv = np.array(flow, float).reshape(-1, 5)
        keep, fkeep = t > -WINDOW_S - 1, ft > -WINDOW_S - 1
        for key, scale, curve in self.curves:
            if key in FLOW_COLS:
                curve.setData(ft[fkeep], fv[fkeep, FLOW_COLS[key]] * scale)
            elif key == "hover":
                b = v[keep, FIELDS.index("battery")]
                curve.setData(t[keep], np.where(b > 5.0, HOVER_V / np.maximum(b, 1e-3), np.nan), connect="finite")
            else:
                curve.setData(t[keep], v[keep, FIELDS.index(key)] * scale)
