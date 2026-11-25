# CameraView - the selected drone's camera: the image (from the slices the M7 sends) and the tracked
# points of the last frame pair, green = inlier, red = outlier, with their motion (x3) since the last frame.

import numpy as np
import pyqtgraph as pg

ARROW = 3.0                             # motion drawn this many times longer


class CameraView(pg.PlotWidget):
    def __init__(self):
        super().__init__(background="k")
        self.setAspectLocked(True)
        self.invertY(True)              # row 0 at the top
        self.hideAxis("left")
        self.hideAxis("bottom")
        self.setMenuEnabled(False)
        self.hideButtons()
        self.image = pg.ImageItem()
        self.addItem(self.image)
        self.motion = {k: pg.PlotDataItem(pen=pg.mkPen(c, width=1)) for k, c in (("in", "#22c55e"), ("out", "#ef4444"))}
        self.dots = {k: pg.ScatterPlotItem(size=7, brush=None, pen=pg.mkPen(c, width=2))
                     for k, c in (("in", "#22c55e"), ("out", "#ef4444"))}
        for item in list(self.motion.values()) + list(self.dots.values()):
            self.addItem(item)
        self.info = pg.TextItem("", color="#facc15", anchor=(0, 0))
        self.addItem(self.info)
        self.shape, self.seq = None, None    # seq: the slice count drawn
        self.setRange(xRange=(0, 160), yRange=(0, 120), padding=0)

    def refresh(self, world, drone):
        if drone is None:
            return
        with world.lock:
            img = drone.image.copy() if drone.image is not None and drone.slices != self.seq else None
            seq, pts, flow = drone.slices, list(drone.points), drone.flow
            flows, slices = drone.flows, drone.slices
        if img is not None:
            self.seq = seq
            self.image.setImage(img, levels=(0, 255), autoLevels=False)
            if img.shape != self.shape:
                self.shape = img.shape
                self.setRange(xRange=(0, img.shape[1]), yRange=(0, img.shape[0]), padding=0)
        for k, sel in (("in", 1), ("out", 0)):
            p = np.array([q for q in pts if bool(q[6]) == bool(sel)], float).reshape(-1, 7)
            self.dots[k].setData(p[:, 2], p[:, 3])
            seg = np.empty((2 * len(p), 2))
            seg[0::2] = p[:, 2:4]
            seg[1::2] = p[:, 2:4] + ARROW * p[:, 4:6]
            self.motion[k].setData(seg[:, 0], seg[:, 1], connect="pairs")
        inl = sum(1 for q in pts if q[6])
        text = f"points {len(pts)}  inliers {inl}   flow packets {flows}  image slices {slices}"
        if flow:
            text += f"\nrms {flow['rmsPx']:.2f} px   valid {flow['valid']}"
        if not slices:
            text += "\nno image yet" if flows else "\nno camera data: m7Runner (PC_WIFI 1) running?"
        self.info.setText(text)

