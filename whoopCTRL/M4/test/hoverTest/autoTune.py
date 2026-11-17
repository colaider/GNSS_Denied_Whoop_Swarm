# Auto-tune the position controller in flight (M4: trajTest.ino, M7: m7Runner with PC_WIFI 1).
#   python autoTune.py NICLA_IP [AXES]    AXES: the order, default zyx (one battery may not last all three:
#                                         each axis is saved when done, run again with the rest, e.g. "x")
# The hoverTest window and keys (C, Enter, L, Space, Esc), plus A = tune (any time: starts once hovering, FLY).
# Its state is the top line of the text and the window title.
# Per axis: still hover (too shaky: kp down first), then small steps on that axis (z 5 cm, x / y 8 cm:
# out, stop, back, stop = one trial) while kp and kd are tried 25 % up / down, the best kept.
# ki follows kp (same ki / kp as at the start). The gains go to the drone at once (PKT_GAINS) and to
# autoTune_gains.json; the lines to paste into PosConfig (CTRL/headers/PosCTRL.h) are printed at the end.

import json
import math
import os
import statistics
import struct
import sys
import time

import hoverTest as H

PKT_GAINS = 22
GAINS = struct.Struct("<12f")           # msg::Gains: kp[4], kd[4], ki[4] (x y z yaw)
PKT_POINT_DEPTH = 23                    # trajTest: the tracked points in 3D (sensors::PointDepth), drawn only
DEPTH_HDR = struct.Struct("<IB")        # msg::PointDepthHeader
DEPTH_PT = struct.Struct("<H3hH")       # msg::PointDepthMsg: id, x y z mm (start frame), depth std mm
DEPTH_SURE, DEPTH_SHOW = 0.03, 0.10     # m: std below = sure (filled), above DEPTH_SHOW not drawn
BOX_M = 0.20                            # point colour: height 0 blue .. this red
AXES = {"x": 0, "y": 1, "z": 2}
STEP_M = {"x": 0.08, "y": 0.08, "z": 0.05}
MOVE_S, STOP_S = 0.8, 3.2               # a step: out in 0.8 s, stop 3.2 s, back, stop: one trial = 8 s
CYCLE = 2 * (MOVE_S + STOP_S)
STILL_S, STILL_SD = 4.0, 0.02           # still hover this long first; error std above 2 cm: kp * 0.8
FACTORS = (1.25, 0.8)                   # tried around the best, kp then kd
ROUNDS = 1                              # 1 round: baseline + 4 trials = 40 s per axis
BETTER = 0.95                           # a trial must be 5 % better to be kept (the measurement is noisy)
ABORT_M = 0.25                          # error larger than this (any axis): stop, best gains back, hold
START_V = 7.6                           # battery (2S, under load) needed to start (log 17.11: 7.1 V, could not hold
                                        # 60 cm). Not checked later: it sags to ~7.2 V on a full one. The camera must
                                        # see the floor (CAM_OK) all along
LIMIT = {"kp": (0.5, 8.0), "kd": (0.5, 6.0)}   # never outside, and never beyond 0.5x .. 2x the start
START = {"kp": [4.5, 4.5, 3.0, 3.0], "kd": [3.4, 3.4, 3.5, 0.0], "ki": [0.9, 0.9, 1.0, 0.0]}   # PosCTRL.h PosConfig
FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "autoTune_gains.json")


class Stop(Exception):                  # tuning stopped inside a step (abort already done)
    pass


def load():
    try:
        with open(FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return json.loads(json.dumps(START))


def save(g):
    with open(FILE, "w") as f:
        json.dump(g, f, indent=1)


def stepFn(axis, amp):                  # one cycle: out amp (half cosine), stop, back, stop
    i = AXES[axis]

    def f(t):
        t %= CYCLE
        p = v = 0.0
        if t < MOVE_S:
            k = t / MOVE_S
            p, v = amp * (1 - math.cos(math.pi * k)) / 2, amp * math.pi / (2 * MOVE_S) * math.sin(math.pi * k)
        elif t < MOVE_S + STOP_S:
            p = amp
        elif t < 2 * MOVE_S + STOP_S:
            k = (t - MOVE_S - STOP_S) / MOVE_S
            p, v = amp * (1 + math.cos(math.pi * k)) / 2, -amp * math.pi / (2 * MOVE_S) * math.sin(math.pi * k)
        out = [0.0] * 7
        out[i], out[4 + i] = p, v
        return tuple(out)
    return f


def cost(e):                            # tracking error over a trial: rms (lag, oscillation) + half the peak (overshoot)
    if len(e) < 50:
        return float("inf")
    return math.sqrt(sum(x * x for x in e) / len(e)) + 0.5 * max(abs(x) for x in e)


class Tuner(H.App):
    def __init__(self, ip, axes):
        self.setup(axes)
        super().__init__(ip)
        self.root.title(f"autoTune {ip}: A = start ({axes})")

    def setup(self, axes):
        self.axes, self.axis, self.job, self.running = axes, None, None, False
        self.gains = load()                             # on the drone now
        self.best = json.loads(json.dumps(self.gains))  # kept so far
        self.samples = []                               # (PC time, error on the axis being tuned)
        self.depth, self.depthTime = [], 0.0            # the last point-depth packet: (id, x, y, z, std) m
        self.status = "A = start auto-tune"

    def sendGains(self):
        g = self.gains
        self.send(PKT_GAINS, GAINS.pack(*g["kp"], *g["kd"], *g["ki"]), 3)

    def setAxis(self, i, kp, kd, ratio):
        self.gains["kp"][i], self.gains["kd"][i], self.gains["ki"][i] = kp, kd, kp * ratio
        self.sendGains()

    def press(self, e):
        k = e.keysym.lower()
        if k == "a" and self.job is None:
            self.job = self.run()
            self.log(f"PC: AUTO-TUNE {self.axes} (starts when hovering, FLY)")
            return
        if self.job is not None and k in ("h", "l", "space", "escape"):
            self.abort("stopped by key")
        super().press(e)

    def onTelem(self, r):
        if self.job is None or not self.running:
            return
        if int(r["state"]) != 4:
            self.abort("not flying (FLY) any more")
            return
        if max(abs(r["t" + a] - r[a]) for a in AXES) > ABORT_M:
            self.abort(f"error over {ABORT_M * 100:.0f} cm")
            return
        if not int(r["flags"]) & H.CAM_OK:
            self.abort("camera lost the floor: more light, more texture on the floor")
            return
        if self.axis:
            self.samples.append((time.time(), r["t" + self.axis] - r[self.axis], r["t" + self.axis]))

    def abort(self, why):
        self.job, self.axis, self.trajT0, self.running = None, None, None, False
        self.gains = json.loads(json.dumps(self.best))
        self.sendGains()
        self.cmd(H.CMD_HOLD, 3)
        save(self.best)
        self.status = f"stopped: {why} (best gains back on, saved)"
        self.log("TUNE " + self.status)

    def step(self):                     # every tick: the tuning runs a little further
        if self.job is not None:
            try:
                next(self.job)
            except (StopIteration, Stop):
                self.job = None

    def tick(self):
        super().tick()
        self.step()
        self.root.title(f"autoTune: {self.status}")

    def onPacket(self, ptype, p):
        if ptype == PKT_POINT_DEPTH and len(p) >= DEPTH_HDR.size:
            n = DEPTH_HDR.unpack_from(p)[1]
            if len(p) == DEPTH_HDR.size + n * DEPTH_PT.size:
                pts = [DEPTH_PT.unpack_from(p, DEPTH_HDR.size + i * DEPTH_PT.size) for i in range(n)]
                self.depth = [(i, x / 1000, y / 1000, z / 1000, s / 1000) for i, x, y, z, s in pts]
                self.depthTime = time.time()

    def drawPos3d(self):                # hoverTest's 3D position view + the tracked points at their height
        super().drawPos3d()
        if not self.depth or time.time() - self.depthTime > 1.0:
            return
        m, h = self.pos3d, H.MAP_M * math.sqrt(2)
        s = min(0.45 * H.VIEW / h, 0.85 * H.VIEW / math.hypot(2 * h, H.POS_Z))
        az, el = self.turn[m]
        pt = lambda q: H.project(q, s, H.VIEW / 2, H.VIEW / 2 + s * H.POS_Z * math.cos(el) / 2, az, el)
        sure = [d for d in self.depth if d[4] < DEPTH_SURE]
        for _, x, y, z, sd in self.depth:
            if sd > DEPTH_SHOW:
                continue
            k = min(max(z / BOX_M, 0.0), 1.0)
            col = "#%02x%02x%02x" % (int(59 + k * (220 - 59)), int(130 - k * 92), int(246 - k * 208))   # blue .. red
            u, v = pt((x, y, z))
            if z > 0.03:
                m.create_line(u, v, *pt((x, y, 0.0)), fill=col, dash=(2, 3))
            if sd < DEPTH_SURE:
                m.create_oval(u - 3, v - 3, u + 3, v + 3, fill=col, outline="")
            else:
                m.create_oval(u - 3, v - 3, u + 3, v + 3, outline=col)
        hs = sorted(d[3] for d in sure)
        m.create_text(6, H.VIEW - 6, anchor="sw", font=("Consolas", 9),
                      text=f"points 3D {len(self.depth)} (sure {len(sure)})" +
                           (f"  height median {hs[len(hs) // 2] * 100:.0f} cm, max {hs[-1] * 100:.0f} cm" if hs else ""))

    def drawText(self):                 # the tuning state on top of hoverTest's text
        super().drawText()
        self.text.config(text=f"AUTO-TUNE ({self.axes}): {self.status}\n" + self.text.cget("text"))

    def run(self):
        while self.last is None or int(self.last[1]) != 4:
            self.status = "waiting for FLY (C, Enter)"
            yield
        r = dict(zip(H.FIELDS, self.last))
        if 1.0 < r["battery"] < START_V:
            self.abort(f"battery {r['battery']:.2f} V: needs {START_V} V to start, land (L) and charge")
            return
        self.running = True
        self.sendGains()
        for axis in self.axes:
            self.axis = axis
            yield from self.tuneAxis(axis)
            save(self.best)
        self.axis, self.running = None, False
        self.status = "DONE: saved, paste into PosCTRL.h PosConfig (L = land)"
        self.log("TUNE " + self.status)
        g = self.best
        for k in ("kp", "kd", "ki"):
            self.log(f"  c.{k} = sensors::Pose{{{', '.join(f'{v:.2f}f' for v in g[k])}}};")

    def tuneAxis(self, axis):
        i, g = AXES[axis], self.gains
        ratio = g["ki"][i] / g["kp"][i] if g["kp"][i] else 0.0
        lo = {k: max(LIMIT[k][0], 0.5 * g[k][i]) for k in LIMIT}
        hi = {k: min(LIMIT[k][1], 2.0 * g[k][i]) for k in LIMIT}

        for n in range(3):              # 1) still hover: shaking already -> softer first
            self.status = f"{axis}: still hover {STILL_S:.0f} s"
            self.samples, t0 = [], time.time()
            while time.time() < t0 + STILL_S:
                yield
            e = [s[1] for s in self.samples]
            sd = statistics.pstdev(e) if len(e) > 20 else 1.0
            self.log(f"TUNE {axis}: still, error std {sd * 100:.1f} cm (kp {g['kp'][i]:.2f})")
            if sd < STILL_SD or g["kp"][i] * 0.8 < lo["kp"]:
                break
            self.setAxis(i, g["kp"][i] * 0.8, g["kd"][i], ratio)
            self.best = json.loads(json.dumps(self.gains))

        self.startTraj(stepFn(axis, STEP_M[axis]))   # 2) steps on this axis
        best = (g["kp"][i], g["kd"][i])
        bestJ = yield from self.trial(i, best, ratio, "baseline")
        for _ in range(ROUNDS):
            for j, name in ((0, "kp"), (1, "kd")):
                for f in FACTORS:
                    cand = list(best)
                    cand[j] = min(max(cand[j] * f, lo[name]), hi[name])
                    if tuple(cand) == best:
                        continue
                    J = yield from self.trial(i, tuple(cand), ratio, f"{name} x{f}")
                    if J < bestJ * BETTER:
                        best, bestJ = tuple(cand), J
                        self.best = json.loads(json.dumps(self.gains))
                        break           # better this way: no need to try the other
        self.setAxis(i, best[0], best[1], ratio)
        self.best = json.loads(json.dumps(self.gains))
        self.trajT0 = None              # the cycle ended back at the hover point: hold there
        self.cmd(H.CMD_HOLD, 3)
        self.log(f"TUNE {axis} done: kp {best[0]:.2f} kd {best[1]:.2f} ki {best[0] * ratio:.2f}  cost {bestJ * 100:.1f} cm")
        t0 = time.time()
        while time.time() < t0 + 2.0:   # settle before the next axis
            yield

    def trial(self, i, gains, ratio, name):
        # gains sent 0.5 s before a cycle starts (drone stopped), measured over that cycle (last 0.6 s left out:
        # the next gains go then)
        t = time.time() - self.trajT0
        b = math.ceil((t + 0.5) / CYCLE) * CYCLE
        while time.time() - self.trajT0 < b - 0.5:
            yield
        self.setAxis(i, gains[0], gains[1], ratio)
        self.samples = []
        self.status = f"{self.axis}: {name}  kp {gains[0]:.2f} kd {gains[1]:.2f}"
        while time.time() - self.trajT0 < b + CYCLE - 0.6:
            yield
        t0 = self.trajT0
        w = [s for s in self.samples if b <= s[0] - t0 < b + CYCLE - 0.6]
        e, target = [s[1] for s in w], [s[2] for s in w]
        if not target or max(target) - min(target) < 0.5 * STEP_M[self.axis]:
            self.abort("the drone did not do the steps: trajTest.ino on the M4? the new m7Runner on the M7?")
            raise Stop
        J = cost(e)
        self.log(f"TUNE {self.axis} {name:<9} kp {gains[0]:.2f} kd {gains[1]:.2f}: cost {J * 100:.1f} cm "
                 f"(peak {max((abs(x) for x in e), default=0) * 100:.1f})")
        return J


if __name__ == "__main__":
    axes = sys.argv[2] if len(sys.argv) == 3 else "zyx"
    if len(sys.argv) in (2, 3) and axes and all(a in AXES for a in axes):
        Tuner(sys.argv[1], axes).root.mainloop()
    else:
        print("python autoTune.py NICLA_IP [zyx]")
