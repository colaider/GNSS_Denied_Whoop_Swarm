# Shows what the M4 sensor test measures, live over USB serial:
#   window "Nicla camera + flow": the camera image with its tracked points (as M7/test/frame_receiver.py)
#     plus the M4's numbers: camera position / velocity, IMU-only velocity, attitude, ToF
#   window "3D position": the FUSED path (green, the Kalman filters), the camera path (cyan: x / y
#     from the derotated flow, z = ToF height), the IMU-only path (orange, it drifts), the drone
#     now (red, on the fused path, line = heading)
#   window "IMU": the last 10 s of the accelerometer (body axes, g, gravity included), the
#     attitude from the gyro (roll / pitch / yaw, deg), and velocity + position from the camera
#     (solid) against the IMU alone (dashed): the gap is how far the IMU has drifted
#   window "Fusion": fused position and velocity (solid) against the camera (dashed); the
#     filters' innovations divided by their std (should stay mostly within +-1, rarely past +-3:
#     NIS = mean of their squares should be ~1 - much more: a noise value is too small, much less:
#     too big); the accelerometer bias the filters found, and the noise of the IMU / ToF now
#   q = quit, c = clear the paths, a / d = turn the 3D view, w / x = tilt it, t = top view,
#   s = save the current frame as PNG into ./frames, p = sanity lines in the terminal on / off
# Sanity check: one line per frame with every step from the flow to the position, printed and
#   always saved to M4/test/logs/sanity_<date>_<time>.txt
# Runs with M7/m7Runner on the M7 and M4/test on the M4.
# pip install pyserial numpy opencv-python matplotlib
# python pose_viewer.py          (auto-finds the Nicla)
# python pose_viewer.py COM5     (or give the port)

import math
import os
import struct
import sys
import time
from collections import deque
import numpy as np

# the packet parser and the camera drawing are shared with the M7 camera test
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "M7", "test"))
from frame_receiver import (PKT_FLOW, PKT_FRAME, PKT_LOG, FlowView, decode_flow,  # noqa: E402
                            decode_frame, find_port, parse_packets)

PKT_SENSORS = 13                                  # same as msg::PKT_SENSORS
SENSORS = struct.Struct("<3IB49f2b2H")            # msg::SensorState (215 bytes)
SENS_CAM_VALID, SENS_IMU_READY, SENS_TOF_VALID, SENS_STILL = 1, 2, 4, 8
FIX_NAMES = ("TRACK", "COAST", "LOST")             # sensors::Fix, flags bits 4-5 (camera) / 6-7 (ToF)

PATH_LEN = 6000                                   # points kept per path (~3 min at 30 Hz)
VIEW_PX = 560                                     # 3D window size
REDRAW_3D_S = 0.12                                # the 3D plot is slow to draw: ~8 times a second
IMU_SECONDS = 10                                  # time shown in the IMU window


def decode_sensors(payload):
    """PKT_SENSORS payload -> dict."""
    v = SENSORS.unpack_from(payload)
    (t, cam_us, seq, flags, cx, cy, cz, cvx, cvy, ix, iy, ivx, ivy,
     roll, pitch, yaw, rng, ax, ay, az, rx, ry, rz, dz, vxc, vyc) = v[:26]
    f = v[26:]                                    # the fused part
    return dict(t=t, cam_us=cam_us, seq=seq, cam_valid=bool(flags & SENS_CAM_VALID),
                imu_ready=bool(flags & SENS_IMU_READY), tof_valid=bool(flags & SENS_TOF_VALID),
                cam=(cx, cy, cz), cam_v=(cvx, cvy), imu=(ix, iy), imu_v=(ivx, ivy),
                roll=roll, pitch=pitch, yaw=yaw, range=rng, acc=(ax, ay, az),
                rot=(rx, ry, rz), dz=dz, cam_v_cam=(vxc, vyc),
                fus=f[0:3], fus_v=f[3:6], std=f[6:9], std_v=f[9:12], bias=f[12:15], acc_w=f[15:18],
                height=f[18], cam_innov=f[19:21], cam_innov_std=f[21:23], tof_innov=f[23],
                tof_innov_std=f[24], cam_std=f[25], tof_std=f[26], cam_result=f[27], tof_result=f[28],
                cam_forced=f[29], tof_steps=f[30], still=bool(flags & SENS_STILL),
                hfix=FIX_NAMES[min(2, (flags >> 4) & 3)], vfix=FIX_NAMES[min(2, (flags >> 6) & 3)])


SANITY_HEADER = ("# per frame: seq | dt = frame pair time, Z = ToF range, dZ = range change over the pair\n"
                 "#   flow = frame motion at the centre (M7, RANSAC), px | inl = inliers / points\n"
                 "#   rot = gyro rotation over the pair, camera axes, deg | rotflow = what that rotation\n"
                 "#   moves the centre by, px (-fx*rot_y, fy*rot_x) | trans = flow - rotflow, px\n"
                 "#   v(tx) = -trans/f * Z/dt (check, from the centre only) | v cam = estimateVelocity,\n"
                 "#   camera axes | v world | pos = camera position, h = height | rpy = attitude, deg | acc, g\n"
                 "#   TRACK/COAST/LOST = camera / ToF state (measuring / missing a moment: IMU alone /\n"
                 "#   missing long: velocity pulled to 0), STILL = IMU sees no motion (velocity 0 if not tracking)\n"
                 "#   FUSED pos / v = the Kalman filters' output, +- = their 1 std | cam in = camera velocity\n"
                 "#   innovation (measured - predicted) / its std, then ok / REFUSED (outlier) / LATE / RESET |\n"
                 "#   tof in = ToF height innovation, mm (/ its std), ok / REFUSED / STEP (floor step) / RESET |\n"
                 "#   bias = accelerometer bias found, m/s^2")


def sanity_line(s, flow):
    """One line with every step from the frame pair to the position (see SANITY_HEADER)."""
    rx, ry, rz = s["rot"]
    head = f"#{s['seq']:<6d}"
    if flow is not None:
        res = flow[0]
        dt = ((res["t1"] - res["t0"]) & 0xFFFFFFFF) * 1e-6
        rfx, rfy = -res["fx"] * ry, res["fy"] * rx                   # rotation flow at the centre, px
        tx, ty = res["tx"] - rfx, res["ty"] - rfy                     # what is left: translation, px
        z = s["range"]
        vtx = (-tx / res["fx"] * z / dt, -ty / res["fy"] * z / dt) if dt > 0 and res["fx"] > 0 else (0.0, 0.0)
        head += (f" dt {dt * 1000:3.0f}ms Z {z:.3f} dZ {s['dz'] * 1000:+5.1f}mm"
                 f" | flow ({res['tx']:+6.2f} {res['ty']:+6.2f})px inl {res['inliers']:2d}/{res['tracked']:<2d}"
                 f" {'' if res['valid'] else 'INVALID '}"
                 f"| rot ({math.degrees(rx):+5.2f} {math.degrees(ry):+5.2f} {math.degrees(rz):+5.2f})deg"
                 f" rotflow ({rfx:+6.2f} {rfy:+6.2f})px trans ({tx:+6.2f} {ty:+6.2f})px"
                 f" | v(tx) ({vtx[0]:+.3f} {vtx[1]:+.3f})")
    else:
        head += " (no flow packet for this frame)"
    return (head +
            f" v cam ({s['cam_v_cam'][0]:+.3f} {s['cam_v_cam'][1]:+.3f}) world ({s['cam_v'][0]:+.3f} {s['cam_v'][1]:+.3f})m/s"
            f" | pos ({s['cam'][0]:+.3f} {s['cam'][1]:+.3f}) h {s['cam'][2]:.3f}"
            f" | rpy ({math.degrees(s['roll']):+6.1f} {math.degrees(s['pitch']):+6.1f} "
            f"{(math.degrees(s['yaw']) + 180) % 360 - 180:+6.1f})"
            f" | acc ({s['acc'][0]:+.2f} {s['acc'][1]:+.2f} {s['acc'][2]:+.2f})"
            f" | {'CAM ok' if s['cam_valid'] else 'cam NOT valid'}{'' if s['tof_valid'] else ' tof NOT valid'}"
            + fused_text(s))


def _ratio(innov, std):
    return innov / std if std > 0 else 0.0


def fused_text(s):
    """The filters' part of the sanity line."""
    fx, fy, fz = s["fus"]
    sx, sy, sz = s["std"]
    cam = {2: "ok", 0: "REFUSED", -1: "LATE", 3: "RESET"}.get(s["cam_result"], "?")
    tof = {1: "ok", 0: "REFUSED", -1: "LATE", 2: "STEP", 3: "RESET"}.get(s["tof_result"], "?")
    return (f" | {s['hfix']}/{s['vfix']}{' STILL' if s['still'] else ''}"
            f" | FUSED pos ({fx:+.3f} {fy:+.3f} {fz:.3f}) +-({sx:.3f} {sy:.3f} {sz:.3f})"
            f" v ({s['fus_v'][0]:+.3f} {s['fus_v'][1]:+.3f} {s['fus_v'][2]:+.3f})"
            f" | cam in ({_ratio(s['cam_innov'][0], s['cam_innov_std'][0]):+5.1f}"
            f" {_ratio(s['cam_innov'][1], s['cam_innov_std'][1]):+5.1f})sd {cam}"
            f" | tof in {s['tof_innov'] * 1000:+5.1f}mm ({_ratio(s['tof_innov'], s['tof_innov_std']):+5.1f}sd) {tof}"
            f" | bias ({s['bias'][0]:+.3f} {s['bias'][1]:+.3f} {s['bias'][2]:+.3f})")


class PoseView:
    """Keeps the camera and IMU paths and draws them in 3D (matplotlib, drawn off-screen)."""

    def __init__(self):
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        self.fig = Figure(figsize=(VIEW_PX / 90, VIEW_PX / 90), dpi=90)
        self.canvas = FigureCanvasAgg(self.fig)
        self.ax = self.fig.add_subplot(projection="3d")
        self.cam = deque(maxlen=PATH_LEN)             # (x, y, z) camera position
        self.imu = deque(maxlen=PATH_LEN)             # (x, y, z) IMU-only position, z from the ToF
        self.fus = deque(maxlen=PATH_LEN)             # (x, y, z) fused position
        self.state = None
        self.last_rx = 0.0
        self.elev, self.azim = 25.0, -60.0

    def clear(self):
        self.cam.clear()
        self.imu.clear()
        self.fus.clear()

    def add(self, s):
        self.state = s
        self.last_rx = time.time()
        if s["imu_ready"]:
            self.fus.append(s["fus"])
        if s["cam_valid"]:
            self.cam.append(s["cam"])
            self.imu.append((s["imu"][0], s["imu"][1], s["cam"][2]))

    def draw(self):
        import cv2
        ax = self.ax
        ax.clear()
        cam = np.array(self.cam) if self.cam else np.zeros((1, 3))
        imu = np.array(self.imu) if self.imu else np.zeros((1, 3))
        fus = np.array(self.fus) if self.fus else np.zeros((1, 3))

        # same metres on x and y: a square around the fused and camera paths (at least 1 m)
        both = np.vstack([cam, fus])
        lo, hi = both[:, :2].min(axis=0), both[:, :2].max(axis=0)
        mid, span = (lo + hi) / 2, max(1.0, float((hi - lo).max()) * 1.3)
        top = max(0.6, float(both[:, 2].max()) * 1.2)
        ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
        ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
        ax.set_zlim(0, top)
        ax.set_box_aspect((1, 1, min(1.0, max(0.25, top / span))))
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("height (m)")
        ax.view_init(elev=self.elev, azim=self.azim)

        ax.scatter([0], [0], [0], c="k", marker="x", s=40)                 # where startup was
        if len(self.imu) > 1:
            ax.plot(imu[:, 0], imu[:, 1], imu[:, 2], color="orange", lw=1, alpha=0.7, label="IMU only")
        if len(self.cam) > 1:
            ax.plot(cam[:, 0], cam[:, 1], cam[:, 2], color="c", lw=1, label="camera")
        if len(self.fus) > 1:
            ax.plot(fus[:, 0], fus[:, 1], fus[:, 2], color="g", lw=2, label="fused")
            ax.plot(fus[:, 0], fus[:, 1], 0 * fus[:, 2], color="0.75", lw=1)   # shadow on the floor
        if self.fus and self.state is not None:
            x, y, z = fus[-1]
            ax.scatter([x], [y], [z], c="r", s=40)
            ax.plot([x, x], [y, y], [0, z], color="r", lw=0.8, ls=":")      # height line
            yaw, k = self.state["yaw"], span * 0.12
            ax.plot([x, x + k * math.cos(yaw)], [y, y + k * math.sin(yaw)], [z, z], color="r", lw=2)
            ax.set_title(f"fused ({x:+.2f}, {y:+.2f}, {z:.2f}) m", fontsize=10)
        if len(self.cam) > 1 or len(self.fus) > 1:
            ax.legend(loc="upper left", fontsize=8)

        self.canvas.draw()
        rgba = np.asarray(self.canvas.buffer_rgba())
        return cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)

    def text_lines(self):
        """The M4's numbers, for the camera window."""
        s = self.state
        if s is None or time.time() - self.last_rx > 1.0:
            return ["M4: no sensor data (is M4/test running?)"]
        cx, cy, cz = s["cam"]
        lines = [f"M4 camera {'VALID' if s['cam_valid'] else 'not valid'}  "
                 f"imu {'ok' if s['imu_ready'] else 'NOT READY'}  tof {'ok' if s['tof_valid'] else 'not valid'}",
                 f"pos ({cx:+.2f}, {cy:+.2f}) m  height {cz:.3f} m  (range {s['range']:.3f})",
                 f"vel camera ({s['cam_v'][0]:+.2f}, {s['cam_v'][1]:+.2f})  imu ({s['imu_v'][0]:+.2f}, "
                 f"{s['imu_v'][1]:+.2f}) m/s",
                 f"roll {math.degrees(s['roll']):+.1f}  pitch {math.degrees(s['pitch']):+.1f}  "
                 f"yaw {(math.degrees(s['yaw']) + 180) % 360 - 180:+.1f} deg",
                 f"FUSED [{s['hfix']}/{s['vfix']}{' STILL' if s['still'] else ''}] "
                 f"({s['fus'][0]:+.2f}, {s['fus'][1]:+.2f}, {s['fus'][2]:.3f}) m  "
                 f"v ({s['fus_v'][0]:+.2f}, {s['fus_v'][1]:+.2f}, {s['fus_v'][2]:+.2f}) m/s"]
        return lines


class ImuPlot:
    """Scrolling plots, drawn with OpenCV: the accelerometer (body axes, g), the attitude (deg),
    and the velocity and position from the camera (solid) against the IMU alone (dashed),
    world axes - the gap between them is how far the IMU has drifted."""

    W, H = 640, 180                                  # one plot
    RED, GREEN, BLUE = (0, 0, 230), (0, 160, 0), (230, 90, 0)          # BGR
    ORANGE, OLIVE = (0, 140, 255), (40, 130, 110)

    def __init__(self):
        self.t = deque()                             # board time, s
        self.acc = deque()                           # (x, y, z), g
        self.att = deque()                           # (roll, pitch, yaw), deg
        self.vel = deque()                           # (camera vx, vy, IMU vx, vy), m/s
        self.pos = deque()                           # (camera x, y, height, IMU x, y), m

    def clear(self):
        for d in (self.t, self.acc, self.att, self.vel, self.pos):
            d.clear()

    def add(self, s):
        t = s["t"] / 1e6
        if self.t and t < self.t[-1] - 1:            # the board restarted (or its clock wrapped)
            self.clear()
        self.t.append(t)
        self.acc.append(s["acc"])
        self.att.append(tuple((math.degrees(v) + 180) % 360 - 180 for v in (s["roll"], s["pitch"], s["yaw"])))
        self.vel.append((s["cam_v"][0], s["cam_v"][1], s["imu_v"][0], s["imu_v"][1]))
        cx, cy, cz = s["cam"]                        # camera: holds its last value while not valid
        self.pos.append((cx, cy, cz, s["imu"][0], s["imu"][1]))
        while self.t[-1] - self.t[0] > IMU_SECONDS:
            for d in (self.t, self.acc, self.att, self.vel, self.pos):
                d.popleft()

    def _plot(self, title, values, names, colours, unit, min_span, wrap=None, dashed=(), guides=()):
        import cv2
        img = np.full((self.H, self.W, 3), 255, np.uint8)
        L, R, T, B = 52, self.W - 150, 24, self.H - 22   # plot area; the legend sits right of R
        cv2.putText(img, title, (L, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        if not self.t:
            cv2.putText(img, "waiting for M4 data", (L + 10, (T + B) // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (140, 140, 140), 1, cv2.LINE_AA)
            return img

        v = np.array(values)                         # rows: samples, columns: the lines
        lo, hi = float(v.min()), float(v.max())
        if hi - lo < min_span:                       # at least min_span tall, centred on the data
            mid = (hi + lo) / 2
            lo, hi = mid - min_span / 2, mid + min_span / 2
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        now = self.t[-1]

        def px(t, y):
            return (int(L + (t - (now - IMU_SECONDS)) / IMU_SECONDS * (R - L)), int(B - (y - lo) / (hi - lo) * (B - T)))

        cv2.rectangle(img, (L, T), (R, B), (210, 210, 210), 1)
        for k in range(5):                           # grid and its labels
            y = lo + (hi - lo) * k / 4
            yy = px(now, y)[1]
            cv2.line(img, (L, yy), (R, yy), (235, 235, 235), 1)
            cv2.putText(img, f"{y:+.2f}" if min_span < 10 else f"{y:+.0f}", (4, yy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)
        if lo < 0 < hi:
            cv2.line(img, (L, px(now, 0)[1]), (R, px(now, 0)[1]), (190, 190, 190), 1)
        for g in guides:                             # reference levels (e.g. +-1 std), dotted
            if lo < g < hi:
                yy = px(now, g)[1]
                for x0 in range(L, R, 8):
                    cv2.line(img, (x0, yy), (x0 + 3, yy), (150, 150, 150), 1)
        cv2.putText(img, f"-{IMU_SECONDS} s", (L, self.H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
        cv2.putText(img, "now", (R - 24, self.H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

        for c in range(v.shape[1]):
            pts = [px(t, y) for t, y in zip(self.t, v[:, c])]
            for i in range(1, len(pts)):
                if wrap and abs(v[i, c] - v[i - 1, c]) > wrap:   # yaw jumps -180 <-> +180: no line across
                    continue
                if c in dashed and (i // 2) % 2:                 # dashed: every other pair of samples
                    continue
                cv2.line(img, pts[i - 1], pts[i], colours[c], 1, cv2.LINE_AA)
            text = f"{names[c]} {v[-1, c]:+.2f} {unit}" if min_span < 10 else f"{names[c]} {v[-1, c]:+.1f} {unit}"
            cv2.putText(img, text, (R + 8, T + 16 + 18 * c), cv2.FONT_HERSHEY_SIMPLEX, 0.42, colours[c], 1, cv2.LINE_AA)
        return img

    def draw(self):
        acc = self._plot("accelerometer, body axes (gravity included)", self.acc, "xyz",
                         [self.RED, self.GREEN, self.BLUE], "g", 0.5)
        att = self._plot("attitude from the gyro", self.att, ["roll", "pitch", "yaw"],
                         [self.RED, self.GREEN, self.BLUE], "deg", 20, wrap=180)
        vt = pt = ""
        if self.t:
            vx, vy, ivx, ivy = self.vel[-1]
            x, y, _, ix, iy = self.pos[-1]
            vt = f" - IMU off by {math.hypot(ivx - vx, ivy - vy):.2f} m/s"
            pt = f" - IMU off by {math.hypot(ix - x, iy - y):.2f} m"
        vel = self._plot("velocity, world: camera (solid) vs IMU alone (dashed)" + vt, self.vel,
                         ["cam x", "cam y", "imu x", "imu y"], [self.RED, self.GREEN, self.ORANGE, self.OLIVE],
                         "m/s", 0.2, dashed=(2, 3))
        pos = self._plot("position, world: camera (solid) vs IMU alone (dashed)" + pt, self.pos,
                         ["cam x", "cam y", "height", "imu x", "imu y"],
                         [self.RED, self.GREEN, self.BLUE, self.ORANGE, self.OLIVE], "m", 0.2, dashed=(3, 4))
        return np.vstack([acc, att, vel, pos])


class FusionPlot(ImuPlot):
    """Scrolling plots of the filters: fused position / velocity (solid) against the camera
    (dashed), the innovations / their std, the bias found and the sensor noise now."""

    MAGENTA = (200, 0, 200)
    NOISE_S = 1.0                                    # live noise: std over the last second

    def __init__(self):
        super().__init__()
        self.fpos = deque()                          # (x, y, z fused, camera x, y, height)
        self.fvel = deque()                          # (vx, vy, vz fused, camera vx, vy)
        self.innov = deque()                         # (camera x, y, ToF) innovation / std
        self.bias = deque()                          # (x, y, z), m/s^2
        self.raw = deque()                           # (acc world x, y, z, ToF height): for the noise
        self.last = None

    def _all(self):
        return (self.t, self.fpos, self.fvel, self.innov, self.bias, self.raw)

    def clear(self):
        for d in self._all():
            d.clear()

    def add(self, s):
        t = s["t"] / 1e6
        if self.t and t < self.t[-1] - 1:            # the board restarted (or its clock wrapped)
            self.clear()
        self.last = s
        self.t.append(t)
        self.fpos.append(tuple(s["fus"]) + tuple(s["cam"]))
        self.fvel.append(tuple(s["fus_v"]) + tuple(s["cam_v"]))
        cam_ok = s["cam_valid"] and s["cam_result"] >= 0
        self.innov.append((_ratio(s["cam_innov"][0], s["cam_innov_std"][0]) if cam_ok else 0.0,
                           _ratio(s["cam_innov"][1], s["cam_innov_std"][1]) if cam_ok else 0.0,
                           _ratio(s["tof_innov"], s["tof_innov_std"])))
        self.bias.append(tuple(s["bias"]))
        self.raw.append(tuple(s["acc_w"]) + (s["height"],))
        while self.t[-1] - self.t[0] > IMU_SECONDS:
            for d in self._all():
                d.popleft()

    def draw(self):
        pt = vt = it = bt = ""
        if self.t:
            s = self.last
            pt = f" - std ({s['std'][0] * 100:.1f} {s['std'][1] * 100:.1f} {s['std'][2] * 100:.1f}) cm"
            vt = f" - std ({s['std_v'][0] * 1000:.0f} {s['std_v'][1] * 1000:.0f} {s['std_v'][2] * 1000:.0f}) mm/s"
            nis = (np.array(self.innov) ** 2).mean(axis=0)
            it = (f" - NIS cam ({nis[0]:.1f} {nis[1]:.1f}) tof {nis[2]:.1f} (want ~1),"
                  f" {s['hfix']}/{s['vfix']}{' STILL' if s['still'] else ''}")
            n = sum(1 for t in self.t if t > self.t[-1] - self.NOISE_S)
            r = np.array(list(self.raw)[-n:])
            if len(r) > 2:                           # meaningful while the board lies still
                sd = r.std(axis=0)
                bt = (f" - noise {self.NOISE_S:.0f} s: acc ({sd[0] / 9.81e-3:.0f} {sd[1] / 9.81e-3:.0f}"
                      f" {sd[2] / 9.81e-3:.0f}) mg, tof {sd[3] * 1000:.1f} mm (given {s['tof_std'] * 1000:.1f})")
        pos = self._plot("position: fused (solid) vs camera (dashed)" + pt, self.fpos,
                         ["x", "y", "z", "cam x", "cam y", "cam h"],
                         [self.RED, self.GREEN, self.BLUE, self.ORANGE, self.OLIVE, self.MAGENTA], "m", 0.2,
                         dashed=(3, 4, 5))
        vel = self._plot("velocity: fused (solid) vs camera (dashed)" + vt, self.fvel,
                         ["vx", "vy", "vz", "cam vx", "cam vy"],
                         [self.RED, self.GREEN, self.BLUE, self.ORANGE, self.OLIVE], "m/s", 0.2, dashed=(3, 4))
        inn = self._plot("innovation / std" + it, self.innov, ["cam x", "cam y", "tof"],
                         [self.RED, self.GREEN, self.BLUE], "sd", 6.5, guides=(-3, -1, 1, 3))
        bia = self._plot("acc bias found, m/s^2" + bt, self.bias, ["x body", "y body", "z"],
                         [self.RED, self.GREEN, self.BLUE], "", 0.1)
        return np.vstack([pos, vel, inn, bia])


def put_lines(img, lines, y0):
    import cv2
    for i, t in enumerate(lines):
        y = y0 + 18 * i
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1, cv2.LINE_AA)


class App:
    """Everything except the serial port and the keys, so it can be fed packets for testing."""

    def __init__(self):
        self.flow_view = FlowView()
        self.pose = PoseView()
        self.frame = None
        self.flows = {}                               # seq -> (result, points)
        self.history = []                             # (seq, t_end_us) of recent frames
        self.view3d = None
        self.last3d = 0.0
        self.imu_plot = ImuPlot()
        self.view_imu = None
        self.last_imu = 0.0
        self.fusion_plot = FusionPlot()
        self.view_fusion = None
        self.sanity = []                              # new sanity lines, taken by main()

    def handle(self, ptype, payload):
        if ptype == PKT_LOG:
            print("[board]", payload.decode(errors="replace"))
        elif ptype == PKT_FRAME:
            info, img = decode_frame(payload)
            self.history = (self.history + [(info["seq"], info["t_end"])])[-30:]
            self.frame = (info, img)
        elif ptype == PKT_FLOW:
            res, pts = decode_flow(payload)
            self.flows[res["seq"]] = (res, pts)
            for old in [s for s in self.flows if s < res["seq"] - 5]:
                del self.flows[old]
            self.flow_view.add(res, pts)
        elif ptype == PKT_SENSORS:
            s = decode_sensors(payload)
            self.pose.add(s)
            self.imu_plot.add(s)
            self.fusion_plot.add(s)
            self.sanity.append(sanity_line(s, self.flows.get(s["seq"])))

    def camera_view(self):
        if self.frame is None:
            return None
        info, img = self.frame
        fps = 0.0
        if len(self.history) > 1:
            dseq = self.history[-1][0] - self.history[0][0]
            dt = ((self.history[-1][1] - self.history[0][1]) & 0xFFFFFFFF) / 1e6   # board clock, wrap-safe
            fps = dseq / dt if dt > 0 else 0.0
        view = self.flow_view.draw_frame(img, info, self.flows.get(info["seq"]), fps)
        lines = self.pose.text_lines()
        put_lines(view, lines, view.shape[0] - 10 - 18 * (len(lines) - 1))
        return view

    def view_3d(self, force=False):
        if force or self.view3d is None or time.time() - self.last3d > REDRAW_3D_S:
            self.view3d = self.pose.draw()
            self.last3d = time.time()
        return self.view3d

    def imu_view(self, force=False):
        if force or self.view_imu is None or time.time() - self.last_imu > 0.05:   # OpenCV: cheap
            self.view_imu = self.imu_plot.draw()
            self.view_fusion = self.fusion_plot.draw()
            self.last_imu = time.time()
        return self.view_imu

    def fusion_view(self):                           # drawn together with the IMU plots
        if self.view_fusion is None:
            self.imu_view(force=True)
        return self.view_fusion


def start_reader(ser):
    """Reads the port in a background thread, always, so the board never waits for us: the
    plots take ~100 ms to draw, and while nobody reads, USB stalls the M7's writes (and with
    them its camera loop). Returns (take, error): take() hands over what arrived so far."""
    import threading
    rx, lock, error = bytearray(), threading.Lock(), []

    def run():
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    with lock:
                        rx.extend(data)
        except Exception as e:                       # port gone (board reset or unplugged)
            error.append(e)

    def take():
        with lock:
            data = bytes(rx)
            rx.clear()
        return data

    threading.Thread(target=run, daemon=True).start()
    return take, error


def main():
    import cv2
    import serial

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, 921600, timeout=0.05)
    print(f"listening on {port} - q quits, c clears, a/d/w/x turn the 3D view, t top view, s saves a frame,"
          f" p sanity lines on/off")

    logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(logs, exist_ok=True)
    log_path = os.path.join(logs, time.strftime("sanity_%Y%m%d_%H%M%S.txt"))
    log = open(log_path, "w", encoding="utf-8")
    log.write(SANITY_HEADER + "\n")
    print(f"sanity lines go to {log_path}")
    print(SANITY_HEADER)
    show_sanity, printed = True, 0

    app = App()
    buf = bytearray()
    take, port_error = start_reader(ser)
    last_rx, warned = time.time(), False

    while True:
        if port_error:
            print(f"serial port error: {port_error[0]} - board reset or unplugged? Start me again.")
            break
        buf += take()
        for ptype, payload in parse_packets(buf):
            last_rx, warned = time.time(), False
            app.handle(ptype, payload)
        for line in app.sanity:
            log.write(line + "\n")
            if show_sanity:
                print(line)
                printed += 1
                if printed % 40 == 0:                # repeat the legend now and then
                    print(SANITY_HEADER)
        app.sanity.clear()
        log.flush()

        if time.time() - last_rx > 3 and not warned:
            print("no packets for 3 s - is m7Runner running and Serial Monitor closed?")
            warned = True

        view = app.camera_view()
        if view is not None:
            cv2.imshow("Nicla camera + flow", view)
        cv2.imshow("3D position", app.view_3d())
        cv2.imshow("IMU", app.imu_view())
        cv2.imshow("Fusion", app.fusion_view())

        key = cv2.waitKey(5) & 0xFF                  # 5 ms: the reader thread does the waiting
        if key == ord("q"):
            break
        if key == ord("p"):
            show_sanity = not show_sanity
            print("sanity lines in the terminal", "on" if show_sanity else "off (still saved to the log)")
        if key == ord("c"):
            app.flow_view.clear()
            app.pose.clear()
            app.imu_plot.clear()
            app.fusion_plot.clear()
        if key in (ord("a"), ord("d")):
            app.pose.azim += 15 if key == ord("a") else -15
            app.view_3d(force=True)
        if key in (ord("w"), ord("x")):
            app.pose.elev = min(90.0, max(0.0, app.pose.elev + (10 if key == ord("w") else -10)))
            app.view_3d(force=True)
        if key == ord("t"):
            app.pose.elev, app.pose.azim = 90.0, -90.0
            app.view_3d(force=True)
        if key == ord("s") and app.frame is not None:
            os.makedirs("frames", exist_ok=True)
            path = os.path.join("frames", f"frame_{app.frame[0]['seq']:06d}.png")
            cv2.imwrite(path, app.frame[1])
            print("saved", path)

    ser.close()
    log.close()
    print(f"sanity log saved: {log_path}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
