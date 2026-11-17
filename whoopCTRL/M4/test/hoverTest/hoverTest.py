# Hover test (M4: hoverTest.ino or trajTest.ino, M7: m7Runner with PC_WIFI 1).
#   python hoverTest.py NICLA_IP          window (camera + points, position, status) + CSV log in logs/
#   python hoverTest.py --plot FILE.csv   plot a flight (+ camera flow quality from FILE_flow.csv)
# Keys (click the window): C = calibrate (still 2 s, position 0 here), Enter = arm + hover,
#                          T = trajectory (trajTest.ino, hovering), H = stop it there,
#                          L = land, Space = KILL, Esc = kill + quit

import binascii
import csv
import math
import os
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque

PORT = 4210
PKT_LOG, PKT_FLOW, PKT_FLIGHT_CMD, PKT_HOVER_TELEM, PKT_FRAME_ROWS = 2, 5, 11, 17, 18
PKT_SEG, PKT_TRAJ_START = 6, 9
CMD_HOLD, CMD_LAND, CMD_KILL, CMD_START, CMD_PING, CMD_CALIBRATE = 1, 2, 4, 5, 6, 7
STATES = ["BOOT", "READY", "SPINUP", "CLIMB", "FLY", "SAFETY", "LAND"]
TELEM_FMT = "<I2B28f2BH2f"                # msg::HoverTelem
FIELDS = ["tUs", "state", "flags", "x", "y", "z", "yaw", "vx", "vy", "vz", "vyaw",
          "tx", "ty", "tz", "tyaw", "roll", "pitch", "yawRate", "throttle",
          "ix", "iy", "iz", "ax", "ay", "az", "attRoll", "attPitch", "battery",
          "tofRange", "tofHeight", "tofSignal", "tofValid", "tofStatus", "floorSteps",
          "camYawRate", "gyroYawRate"]
ARMED, CAM_OK, TOF_OK, FC_LINK, CALIBRATED = 1, 2, 4, 8, 16
FLOW_HDR = struct.Struct("<4I9f3HB")    # msg::FlowHeader
POINT = struct.Struct("<2H4fB")         # msg::PointMsg
ROWS_HDR = struct.Struct("<I3HB")       # msg::FrameRows
SEG_HDR = struct.Struct("<2HB")         # msg::SegHeader
TRAJ_PT = struct.Struct("<I7f")         # msg::TrajPoint
TRAJ_START = struct.Struct("<HI")       # msg::TrajStart
FLOW_FIELDS = ["t1Us", "seq", "valid", "tracked", "inliers", "rmsPx", "tx", "ty", "angleDeg", "scale",
               "flowUs", "movePx", "spreadPx", "state", "armed"]

CAM_W, CAM_H = 160, 120                 # camera image (m7Runner crop: the whole 320x240 view, binned 2x2)
ZOOM = 3                                # camera: 160x120 -> 480x360
MAP = 400                               # position view, pixels
MAP_M = 0.6                             # position view: +-0.6 m (the 40 cm moves fit)
Z_MAX = 0.3                             # height bar: 0..0.3 m
VIEW = 400                              # 3D views (attitude, position), pixels
AZ, EL = math.radians(225), math.radians(25)   # 3D views at start: seen from behind-right (-x, -y), above; drag to turn
POS_Z = 1.0                             # 3D position view: 0..1 m high

TRAJ_DT = 0.2                           # trajectory: a point every 0.2 s...
TRAJ_POINTS = 6                         # ...6 per packet (msg::SEG_MAX_POINTS): 1 s ahead, sent every tick
TRAJ_DELAY = 0.5                        # s: the drone starts this long after the start packet
MOVE_M, MOVE_S, STOP_S = 0.40, 1.5, 5.0   # forward 40 cm in 1.5 s (peak 0.42 m/s, 0.9 m/s^2 ~ 5 deg), stop 5 s, back, again


def packet(ptype, payload):
    head = struct.pack("<BI", ptype, len(payload))
    return b"\xA5\x5A" + head + payload + struct.pack("<H", binascii.crc_hqx(head + payload, 0xFFFF))


def project(p, s, cu, cv, az, el):      # 3D views: start-frame point (x forward, y left, z up) -> canvas
    x, y, z = p
    right = -x * math.sin(az) + y * math.cos(az)
    up = -math.sin(el) * (x * math.cos(az) + y * math.sin(az)) + z * math.cos(el)
    return cu + s * right, cv - s * up


def rotate(roll, pitch, yaw, v):        # body -> start frame, ZYX like IMU::Attitude, radians
    x, y, z = v
    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
    return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z


def forwardBack(t):                     # trajectory at time t: (x, y, z, yaw, vx, vy, vz), from the hover point
    t %= 2 * (MOVE_S + STOP_S)
    if t < MOVE_S:                      # forward, smooth start and stop (half cosine)
        k = t / MOVE_S
        return MOVE_M * (1 - math.cos(math.pi * k)) / 2, 0, 0, 0, MOVE_M * math.pi / (2 * MOVE_S) * math.sin(math.pi * k), 0, 0
    if t < MOVE_S + STOP_S:
        return MOVE_M, 0, 0, 0, 0, 0, 0
    if t < 2 * MOVE_S + STOP_S:         # back
        k = (t - MOVE_S - STOP_S) / MOVE_S
        return MOVE_M * (1 + math.cos(math.pi * k)) / 2, 0, 0, 0, -MOVE_M * math.pi / (2 * MOVE_S) * math.sin(math.pi * k), 0, 0
    return 0, 0, 0, 0, 0, 0, 0


def parse(buf):
    out = []
    while True:
        i = buf.find(b"\xA5\x5A")
        if i < 0:
            del buf[:-1]
            return out
        if len(buf) < i + 7:
            del buf[:i]
            return out
        ptype, n = struct.unpack_from("<BI", buf, i + 2)
        if n > 2048:
            del buf[:i + 2]
            continue
        if len(buf) < i + 9 + n:
            del buf[:i]
            return out
        body = bytes(buf[i + 2:i + 7 + n])
        if binascii.crc_hqx(body, 0xFFFF) == struct.unpack_from("<H", buf, i + 7 + n)[0]:
            out.append((ptype, body[5:]))
            del buf[:i + 9 + n]
        else:
            del buf[:i + 2]


def plot(path):
    import matplotlib.pyplot as plt
    with open(path) as f:
        rows = list(csv.DictReader(f))
    col = lambda k: [float(r[k]) for r in rows]
    t = [(x - col("tUs")[0]) * 1e-6 for x in col("tUs")]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    ax[0].plot(t, col("z"), label="z")
    ax[0].plot(t, col("tz"), "--", label="target")
    ax[0].set_ylabel("m")
    ax[1].plot(t, col("throttle"), label="throttle")
    ax[1].plot(t, [v / 50.0 for v in col("iz")], label="z integral (throttle)")
    ax[2].plot(t, col("x"), label="x")
    ax[2].plot(t, col("y"), label="y")
    ax[2].set_ylabel("m")
    ax[3].plot(t, col("roll"), label="roll cmd")
    ax[3].plot(t, col("attRoll"), label="roll IMU")
    ax[3].plot(t, col("pitch"), label="pitch cmd")
    ax[3].plot(t, col("attPitch"), label="pitch IMU")
    ax[3].set_ylabel("deg")
    ax[3].set_xlabel("s")
    for a in ax:
        a.legend(loc="upper right")
        a.grid(True)
    plt.tight_layout()

    fpath = path.replace(".csv", "_flow.csv")
    if os.path.exists(fpath):
        with open(fpath) as f:
            fr = list(csv.DictReader(f))
        fc = lambda k: [float(r[k]) for r in fr]
        ft = [(x - col("tUs")[0]) * 1e-6 for x in fc("t1Us")]
        fly = [r["armed"] == "1" and r["state"] in ("2", "3", "4") for r in fr]
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
        fig.suptitle("camera flow (grey = flying)")
        ax[0].plot(ft, fc("tracked"), label="tracked points")
        ax[0].plot(ft, fc("inliers"), label="inliers")
        ax[1].plot(ft, fc("rmsPx"), label="RANSAC rms (inliers)")
        ax[1].plot(ft, fc("spreadPx"), label="spread: points disagree")
        ax[1].set_ylabel("px")
        ax[2].plot(ft, fc("movePx"), label="mean point motion per frame")
        ax[2].plot(ft, [v * 5 for v in fc("valid")], label="valid (x5)")
        ax[2].set_ylabel("px")
        ax[2].set_xlabel("s")
        spans, start = [], None             # flying stretches, one grey band each
        for i, f in enumerate(fly + [False]):
            if f and start is None:
                start = i
            elif not f and start is not None:
                spans.append((ft[start], ft[i - 1]))
                start = None
        for a in ax:
            for s0, s1 in spans:
                a.axvspan(s0, s1, color="#dddddd", lw=0)
            a.legend(loc="upper right")
            a.grid(True)
        plt.tight_layout()
        for name, sel in (("ground", [not f for f in fly]), ("flying", fly)):
            rs = [r for r, s in zip(fr, sel) if s]
            if rs:
                m = lambda k: sum(float(r[k]) for r in rs) / len(rs)
                print(f"{name:7s} frames {len(rs):5d}  valid {m('valid') * 100:3.0f}%  tracked {m('tracked'):4.1f}  "
                      f"inliers {m('inliers'):4.1f}  rms {m('rmsPx'):.2f} px  spread {m('spreadPx'):.2f} px  "
                      f"motion {m('movePx'):.2f} px/frame")
    plt.show()


class App:
    def __init__(self, ip):
        self.addr = (ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)   # Windows default 8 KB: image slices got lost
        self.last, self.lastTime, self.logs, self.n = None, 0.0, [], 0
        self.img, self.imgW, self.imgH, self.photo = bytearray(), 0, 0, None
        self.points, self.camNew = [], False
        self.flows, self.slices = 0, 0
        self.trail = deque(maxlen=1000)
        self.trajId, self.trajT0, self.segSeq = 0, None, 0   # trajT0: PC time of trajectory t = 0 (None: not sending)
        self.trajFn = forwardBack

        os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"), exist_ok=True)
        self.path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs",
                                 time.strftime("hover_%Y%m%d_%H%M%S.csv"))
        self.file = open(self.path, "w", newline="")
        self.csv = csv.writer(self.file)
        self.csv.writerow(FIELDS)
        self.flowFile = open(self.path.replace(".csv", "_flow.csv"), "w", newline="")
        self.flowCsv = csv.writer(self.flowFile)
        self.flowCsv.writerow(FLOW_FIELDS)

        self.root = tk.Tk()
        self.root.title(f"hoverTest {ip}")
        views = tk.Frame(self.root)
        views.pack(side="top")
        self.cam = tk.Canvas(views, width=CAM_W * ZOOM, height=CAM_H * ZOOM, bg="black", highlightthickness=0)
        self.cam.pack(side="left", padx=8, pady=8)
        self.map = tk.Canvas(views, width=MAP + 60, height=MAP, bg="white", highlightthickness=0)
        self.map.pack(side="left", padx=8, pady=8)
        self.att = tk.Canvas(views, width=VIEW, height=VIEW, bg="white", highlightthickness=0)
        self.att.pack(side="left", padx=8, pady=8)
        self.pos3d = tk.Canvas(views, width=VIEW, height=VIEW, bg="white", highlightthickness=0)
        self.pos3d.pack(side="left", padx=8, pady=8)
        self.turn = {self.att: [AZ, EL], self.pos3d: [AZ, EL]}   # each 3D view's azimuth / elevation
        for c in self.turn:
            c.bind("<ButtonPress-1>", self.grab)
            c.bind("<B1-Motion>", self.drag)
        self.text = tk.Label(self.root, font=("Consolas", 11), justify="left", anchor="nw", width=100, height=16)
        self.text.pack(side="top", padx=8, pady=4)
        self.root.bind("<KeyPress>", self.press)
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(0, self.tick)
        self.alive = True
        threading.Thread(target=self.keepAlive, daemon=True).start()

    def keepAlive(self):                # own thread: a blocked window (dragged, a menu open) stopped the PING -> SAFETY
        while self.alive:
            self.cmd(CMD_PING)
            t0 = self.trajT0
            if t0 is not None:
                self.sendTraj(t0)
            time.sleep(0.05)

    def send(self, ptype, payload, times=1):
        for _ in range(times):
            try:
                self.sock.sendto(packet(ptype, payload), self.addr)
            except OSError:
                pass

    def cmd(self, c, times=1):
        self.send(PKT_FLIGHT_CMD, bytes([c]), times)

    def sendTraj(self, t0):             # the points from just before now to 1 s ahead; repeats are fine
        t = time.time() - t0
        k0 = max(0, int(t / TRAJ_DT))
        pts = b"".join(TRAJ_PT.pack(round(k * TRAJ_DT * 1000), *self.trajFn(k * TRAJ_DT))
                       for k in range(k0, k0 + TRAJ_POINTS))
        self.send(PKT_SEG, SEG_HDR.pack(self.trajId, self.segSeq & 0xFFFF, TRAJ_POINTS) + pts)
        self.segSeq += 1

    def startTraj(self, fn):            # fn(t) -> (x, y, z, yaw, vx, vy, vz) from the hover point
        self.trajFn = fn
        self.trajId = (self.trajId + 1) & 0xFFFF
        self.trajT0 = time.time() + TRAJ_DELAY
        self.send(PKT_TRAJ_START, TRAJ_START.pack(self.trajId, int(TRAJ_DELAY * 1000)), 3)

    def onTelem(self, r):               # every telemetry packet (autoTune.py uses it)
        pass

    def onPacket(self, ptype, p):       # packets this window does not know (autoTune.py: point depth)
        pass

    def log(self, line):
        line = time.strftime("%H:%M:%S ") + line
        print(line)
        self.logs = (self.logs + [line])[-8:]

    def press(self, e):
        k = e.keysym.lower()
        if k == "c":
            self.cmd(CMD_CALIBRATE, 3)
            self.log("PC: CALIBRATE (keep it still)")
        elif k == "return":
            self.cmd(CMD_START, 3)
            self.log("PC: START")
        elif k == "t":
            self.startTraj(forwardBack)
            self.log(f"PC: TRAJECTORY {self.trajId} (forward {MOVE_M * 100:.0f} cm, stop {STOP_S:.0f} s, back, again)")
        elif k == "h":
            self.trajT0 = None
            self.cmd(CMD_HOLD, 3)
            self.log("PC: HOLD (trajectory stopped)")
        elif k == "l":
            self.trajT0 = None
            self.cmd(CMD_LAND, 3)
            self.log("PC: LAND")
        elif k == "space":
            self.trajT0 = None
            self.cmd(CMD_KILL, 3)
            self.log("PC: KILL")
        elif k == "escape":
            self.quit()

    def tick(self):
        packets = []
        while True:
            try:
                packets += parse(bytearray(self.sock.recvfrom(4096)[0]))   # each datagram alone: a cut one can't eat the next
            except OSError:
                break
        for ptype, p in packets:
            if ptype == PKT_HOVER_TELEM and len(p) == struct.calcsize(TELEM_FMT):
                self.last = struct.unpack(TELEM_FMT, p)
                self.lastTime = time.time()
                self.csv.writerow([f"{x:.5f}" if isinstance(x, float) else x for x in self.last])
                self.n += 1
                r = dict(zip(FIELDS, self.last))
                self.trail.append((r["x"], r["y"], r["z"]))
                self.onTelem(r)
            elif ptype == PKT_FRAME_ROWS and len(p) >= ROWS_HDR.size:
                seq, w, h, row0, n = ROWS_HDR.unpack_from(p)
                if (w, h) != (self.imgW, self.imgH):
                    self.img, self.imgW, self.imgH = bytearray(w * h), w, h
                if len(p) == ROWS_HDR.size + n * w and row0 + n <= h:
                    self.img[row0 * w:(row0 + n) * w] = p[ROWS_HDR.size:]
                    self.camNew = True
                    self.slices += 1
            elif ptype == PKT_FLOW and len(p) >= FLOW_HDR.size:
                count = FLOW_HDR.unpack_from(p)[15]
                if len(p) == FLOW_HDR.size + count * POINT.size:
                    self.points = [POINT.unpack_from(p, FLOW_HDR.size + i * POINT.size) for i in range(count)]
                    self.camNew = True
                    self.flows += 1
                    self.logFlow(FLOW_HDR.unpack_from(p))
            elif ptype == PKT_LOG:
                self.log(p.decode(errors="replace"))
            else:
                self.onPacket(ptype, p)
        if self.camNew or not self.imgW:
            self.drawCamera()
        self.drawMap()
        self.drawAttitude()
        self.drawPos3d()
        self.drawText()
        self.root.after(50, self.tick)

    def logFlow(self, h):               # one row per frame: how well the camera tracks
        seq, t0, t1, flowUs, tx, ty, angle, scale, rms, fx, fy, cx, cy, tracked, inliers, count, valid = h
        d = [(p[4], p[5]) for p in self.points]
        n = max(len(d), 1)
        mx, my = sum(a for a, _ in d) / n, sum(b for _, b in d) / n
        move = sum(math.hypot(a, b) for a, b in d) / n
        spread = math.sqrt(sum((a - mx) ** 2 + (b - my) ** 2 for a, b in d) / n)
        r = dict(zip(FIELDS, self.last)) if self.last else {"state": 0, "flags": 0}
        self.flowCsv.writerow([t1, seq, valid, tracked, inliers, f"{rms:.3f}", f"{tx:.3f}", f"{ty:.3f}",
                               f"{math.degrees(angle):.3f}", f"{scale:.4f}", flowUs, f"{move:.3f}", f"{spread:.3f}",
                               int(r["state"]), int(r["flags"]) & ARMED])

    def drawCamera(self):
        self.camNew = False
        self.cam.delete("all")
        if self.imgW:
            pgm = b"P5 %d %d 255\n" % (self.imgW, self.imgH) + bytes(self.img)
            self.photo = tk.PhotoImage(data=pgm, format="PPM").zoom(ZOOM)
            self.cam.create_image(0, 0, image=self.photo, anchor="nw")
        for _, age, x, y, dx, dy, inlier in self.points:       # point + its motion (x3) since the last frame
            c = "lime" if inlier else "red"
            u, v = x * ZOOM, y * ZOOM
            self.cam.create_oval(u - 3, v - 3, u + 3, v + 3, outline=c, width=2)
            self.cam.create_line(u, v, u + dx * ZOOM * 3, v + dy * ZOOM * 3, fill=c)
        inl = sum(1 for p in self.points if p[6])
        self.cam.create_text(6, 6, anchor="nw", fill="yellow", font=("Consolas", 10),
                             text=f"points {len(self.points)}  inliers {inl}   flow packets {self.flows}  image slices {self.slices}")
        if not self.imgW:
            self.cam.create_text(CAM_W * ZOOM / 2, CAM_H * ZOOM / 2, fill="yellow", font=("Consolas", 10), justify="center",
                                 text="no image yet\nupload the NEW m7Runner (PC_WIFI 1)" if self.flows else
                                      "no camera data at all\nm7Runner running? same WiFi?")

    def drawMap(self):                  # top view: x up, y left (start frame), start = centre
        m, s, c = self.map, MAP / (2 * MAP_M), MAP / 2
        px = lambda x, y: (c - y * s, c - x * s)
        m.delete("all")
        for g in range(-4, 5):          # grid every 10 cm
            k = c + g * 0.1 * s
            m.create_line(k, 0, k, MAP, fill="#e4e4e4")
            m.create_line(0, k, MAP, k, fill="#e4e4e4")
        m.create_line(c, 0, c, MAP, fill="#bbbbbb")
        m.create_line(0, c, MAP, c, fill="#bbbbbb")
        m.create_text(c + 4, 4, anchor="nw", text="x", fill="#888888")
        m.create_text(4, c + 4, anchor="nw", text="y", fill="#888888")
        if len(self.trail) > 1:
            m.create_line(*[v for p in self.trail for v in px(p[0], p[1])], fill="#3b82f6")
        if self.last is None:
            return
        r = dict(zip(FIELDS, self.last))
        tu, tv = px(r["tx"], r["ty"])
        m.create_line(tu - 8, tv, tu + 8, tv, fill="red", width=2)
        m.create_line(tu, tv - 8, tu, tv + 8, fill="red", width=2)
        u, v = px(r["x"], r["y"])
        m.create_oval(u - 5, v - 5, u + 5, v + 5, fill="#1d4ed8", outline="")
        hu, hv = px(r["x"] + 0.06 * math.cos(r["yaw"]), r["y"] + 0.06 * math.sin(r["yaw"]))
        m.create_line(u, v, hu, hv, fill="#1d4ed8", width=2)     # heading
        b0, b1 = MAP + 20, MAP + 44     # height bar
        zp = lambda z: MAP - min(max(z, 0.0), Z_MAX) / Z_MAX * MAP
        m.create_rectangle(b0, 0, b1, MAP, outline="#bbbbbb")
        m.create_rectangle(b0, zp(r["z"]), b1, MAP, fill="#93c5fd", outline="")
        m.create_line(b0 - 4, zp(r["tz"]), b1 + 4, zp(r["tz"]), fill="red", width=2)
        m.create_text(b0, 4, anchor="nw", text=f"{r['z'] * 100:.0f}cm", font=("Consolas", 9))

    def grab(self, e):
        self.mouse = (e.x, e.y)

    def drag(self, e):                  # left / right: turn around z, up / down: tilt the view
        v = self.turn[e.widget]
        v[0] -= (e.x - self.mouse[0]) * 0.01
        v[1] = min(max(v[1] + (e.y - self.mouse[1]) * 0.01, -1.5), 1.5)
        self.mouse = (e.x, e.y)
        if e.widget is self.att:
            self.drawAttitude()
        else:
            self.drawPos3d()

    def drawAttitude(self):             # the drone's orientation (IMU attitude + yaw), 3D
        a, s, c = self.att, VIEW * 0.55, VIEW / 2
        pt = lambda p: project(p, s, c, c, *self.turn[a])
        a.delete("all")
        a.create_polygon(*[v for p in [(-.5, -.5, 0), (.5, -.5, 0), (.5, .5, 0), (-.5, .5, 0)] for v in pt(p)],
                         fill="", outline="#dddddd", dash=(4, 3))          # level
        for v, name in (((0.5, 0, 0), "x"), ((0, 0.5, 0), "y"), ((0, 0, 0.5), "z")):
            a.create_line(*pt((0, 0, 0)), *pt(v), fill="#cccccc")
            a.create_text(*pt(v), text=name, fill="#999999")
        r = dict(zip(FIELDS, self.last)) if self.last else {"attRoll": 0.0, "attPitch": 0.0, "yaw": 0.0}
        roll, pitch, yaw = math.radians(r["attRoll"]), math.radians(r["attPitch"]), r["yaw"]
        b = lambda v: pt(rotate(roll, pitch, yaw, v))
        for mx, my in ((-1, -1), (-1, 1), (1, -1), (1, 1)):                 # arms + motor rings, front orange
            col = "#f97316" if mx > 0 else "#64748b"
            m0 = (0.3 * mx, 0.3 * my, 0)
            a.create_line(*b((0, 0, 0)), *b(m0), fill=col, width=4)
            ring = [b((m0[0] + 0.12 * math.cos(k * math.pi / 8), m0[1] + 0.12 * math.sin(k * math.pi / 8), 0)) for k in range(16)]
            a.create_polygon(*[v for p in ring for v in p], fill="", outline=col, width=2)
        a.create_line(*b((0, 0, 0)), *b((0.55, 0, 0)), fill="#dc2626", width=2, arrow="last")   # nose
        a.create_line(*b((0, 0, 0)), *b((0, 0, 0.35)), fill="#2563eb", width=2, arrow="last")   # thrust
        a.create_text(6, 6, anchor="nw", font=("Consolas", 10),
                      text=f"attitude  roll {r['attRoll']:+5.1f}  pitch {r['attPitch']:+5.1f}  yaw {math.degrees(yaw):+6.1f} deg")

    def drawPos3d(self):                # position, 3D: floor grid every 10 cm (+-MAP_M), 0..POS_Z high
        m, h = self.pos3d, MAP_M * math.sqrt(2)
        s = min(0.45 * VIEW / h, 0.85 * VIEW / math.hypot(2 * h, POS_Z))   # fits at any view angle
        az, el = self.turn[m]
        pt = lambda p: project(p, s, VIEW / 2, VIEW / 2 + s * POS_Z * math.cos(el) / 2, az, el)
        m.delete("all")
        n = round(MAP_M / 0.1)
        for g in range(-n, n + 1):
            k = g * 0.1
            m.create_line(*pt((k, -MAP_M, 0)), *pt((k, MAP_M, 0)), fill="#bbbbbb" if g == 0 else "#e4e4e4")
            m.create_line(*pt((-MAP_M, k, 0)), *pt((MAP_M, k, 0)), fill="#bbbbbb" if g == 0 else "#e4e4e4")
        for cx, cy in ((-MAP_M, -MAP_M), (-MAP_M, MAP_M), (MAP_M, -MAP_M), (MAP_M, MAP_M)):
            m.create_line(*pt((cx, cy, 0)), *pt((cx, cy, POS_Z)), fill="#eeeeee")
        m.create_line(*pt((0, 0, 0)), *pt((0, 0, POS_Z)), fill="#bbbbbb")
        for k in range(1, int(POS_Z / 0.2 + 0.5) + 1):                      # height ticks every 20 cm
            m.create_text(*pt((0, 0, k * 0.2)), anchor="w", text=f" {k * 20}cm", fill="#999999", font=("Consolas", 8))
        m.create_text(*pt((MAP_M + 0.08, 0, 0)), text="x", fill="#888888")
        m.create_text(*pt((0, MAP_M + 0.08, 0)), text="y", fill="#888888")
        m.create_text(6, 6, anchor="nw", text="position 3D", font=("Consolas", 10))
        if len(self.trail) > 1:
            m.create_line(*[v for p in self.trail for v in pt(p)], fill="#3b82f6")
        if self.last is None:
            return
        r = dict(zip(FIELDS, self.last))
        for d in ((0.04, 0, 0), (0, 0.04, 0), (0, 0, 0.04)):                # setpoint
            m.create_line(*pt((r["tx"] - d[0], r["ty"] - d[1], r["tz"] - d[2])),
                          *pt((r["tx"] + d[0], r["ty"] + d[1], r["tz"] + d[2])), fill="red", width=2)
        u, v = pt((r["x"], r["y"], r["z"]))
        su, sv = pt((r["x"], r["y"], 0))
        m.create_line(su, sv, u, v, fill="#94a3b8", dash=(3, 3))           # drop line to the floor
        m.create_oval(su - 3, sv - 2, su + 3, sv + 2, fill="#94a3b8", outline="")
        m.create_oval(u - 5, v - 5, u + 5, v + 5, fill="#1d4ed8", outline="")

    def drawText(self):
        if self.last is None or time.time() - self.lastTime > 1.0:
            lines = ["NO TELEMETRY: same WiFi? right IP? m7Runner PC_WIFI 1?"]
        else:
            r = dict(zip(FIELDS, self.last))
            f = int(r["flags"])
            ok = lambda bit: "ok" if f & bit else "--"
            lines = [f"{STATES[int(r['state'])]:<8} {'ARMED' if f & ARMED else 'disarmed':<9} "
                     f"FC {ok(FC_LINK)}  cam {ok(CAM_OK)}  ToF {ok(TOF_OK)}  {r['battery']:.2f} V  "
                     f"{'calibrated' if f & CALIBRATED else 'NOT CALIBRATED (press C)'}",
                     f"z {r['z'] * 100:6.1f} cm  target {r['tz'] * 100:5.1f}  vz {r['vz'] * 100:+6.1f} cm/s",
                     f"x {r['x'] * 100:+6.1f}  y {r['y'] * 100:+6.1f} cm  yaw {r['yaw'] * 57.3:+6.1f} deg",
                     f"throttle {r['throttle']:.3f}  roll {r['roll']:+5.1f}  pitch {r['pitch']:+5.1f}  "
                     f"yaw rate {r['yawRate']:+5.1f}"]
        traj = (f"trajectory {self.trajId}: sending, t {time.time() - self.trajT0:5.1f} s"
                if self.trajT0 is not None else "trajectory: off")
        lines += [traj, f"{self.n} samples -> {self.path}",
                  "C = calibrate (still)   Enter = take off   T = trajectory   H = hold   L = land   "
                  "SPACE = KILL   Esc = quit"] + self.logs
        self.text.config(text="\n".join(lines))

    def quit(self):
        self.alive = False
        self.cmd(CMD_KILL, 5)
        self.file.close()
        self.flowFile.close()
        self.root.destroy()
        print(f"saved {self.path}\nplot: python hoverTest.py --plot \"{self.path}\"")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--plot":
        plot(sys.argv[2])
    elif len(sys.argv) == 2:
        App(sys.argv[1]).root.mainloop()
    else:
        print("python hoverTest.py NICLA_IP  |  python hoverTest.py --plot FILE.csv")
