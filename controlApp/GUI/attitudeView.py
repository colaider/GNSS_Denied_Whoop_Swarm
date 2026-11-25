# AttitudeView - the selected drone's orientation in 3D: solid = measured (IMU roll / pitch + yaw),
# dashed grey = what was sent to Betaflight (roll / pitch command): does the FC follow? Front arms
# orange, nose red, thrust blue. Drag to turn, wheel to zoom.

import math

import numpy as np
from PySide6.QtCore import Qt
import pyqtgraph as pg

from GUI.orbit3d import Orbit3D
from utilities.geometry import ring, rotate

MOTORS = [(0.3 * mx, 0.3 * my, 0.0) for mx in (-1, 1) for my in (-1, 1)]


def body():                             # the drone, body frame: arms (pairs), rings (closed, NaN-separated)
    arms = np.array([p for m in MOTORS for p in ((0, 0, 0), m)])
    rings = np.vstack([np.vstack([ring(m, 0.12), (np.nan, np.nan, np.nan)]) for m in MOTORS])
    return arms, rings


class AttitudeView(Orbit3D):
    def __init__(self):
        super().__init__(radius=0.75)
        self.level = self.line("#dddddd", 1, Qt.DashLine)
        self.axes = self.line("#cccccc")
        self.cmdArms = self.line("#bbbbbb", 2, Qt.DashLine)
        self.armsBack = self.line("#64748b", 4)
        self.armsFront = self.line("#f97316", 4)
        self.rings = self.line("#64748b", 2)
        self.nose = self.line("#dc2626", 2)
        self.thrust = self.line("#2563eb", 2)
        self.info = pg.TextItem("", color="#333333", anchor=(0, 0))
        self.addItem(self.info)
        self.arms, self.ringPts = body()

    def setRings(self, pts):            # NaN gaps between the rings
        u, v = self.xy(pts)
        self.rings.setData(u, v, connect="finite")

    def refresh(self, world, drone):
        with world.lock:
            r = dict(drone.telem) if drone is not None and drone.telem else None
        self.setLine(self.level, np.array([(-.5, -.5, 0), (.5, -.5, 0), (.5, .5, 0), (-.5, .5, 0), (-.5, -.5, 0)]))
        self.setLine(self.axes, np.array([(0, 0, 0), (.5, 0, 0), (0, 0, 0), (0, .5, 0), (0, 0, 0), (0, 0, .5)]), pairs=True)
        roll, pitch, yaw = (math.radians(r["attRoll"]), math.radians(r["attPitch"]), r["yaw"]) if r else (0.0, 0.0, 0.0)
        b = lambda v: rotate(roll, pitch, yaw, v)
        if r:
            self.setLine(self.cmdArms, rotate(math.radians(r["roll"]), math.radians(r["pitch"]), yaw, self.arms), pairs=True)
        arms = b(self.arms)
        front = arms[[i for i in range(len(arms)) if MOTORS[i // 2][0] > 0]]
        back = arms[[i for i in range(len(arms)) if MOTORS[i // 2][0] < 0]]
        self.setLine(self.armsFront, front, pairs=True)
        self.setLine(self.armsBack, back, pairs=True)
        self.setRings(b(self.ringPts))
        self.setLine(self.nose, b([(0, 0, 0), (0.55, 0, 0)]))
        self.setLine(self.thrust, b([(0, 0, 0), (0, 0, 0.35)]))
        (x0, _), (_, y1) = self.plotItem.vb.viewRange()
        self.info.setPos(x0, y1)
        self.info.setText(f"roll  {r['attRoll']:+5.1f}  (sent {r['roll']:+5.1f}) deg\n"
                          f"pitch {r['attPitch']:+5.1f}  (sent {r['pitch']:+5.1f}) deg\n"
                          f"yaw   {math.degrees(r['yaw']):+6.1f} deg" if r else "attitude: no telemetry")
