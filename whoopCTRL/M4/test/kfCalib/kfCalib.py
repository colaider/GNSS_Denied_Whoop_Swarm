# Kalman filter calibration (M4: kfCalib.ino, M7: m7Runner with PC_WIFI 1). Windows (keys).
#   python kfCalib.py NICLA_IP        guided recording (still / slow / fast) -> logs/kf_*.npz, then the fit
#   python kfCalib.py --fit FILE.npz  fit a recording again
# The fit: noise of the sensors while still (Allan deviation), then both filters run again offline
# on the recording and their noise values are fitted (most likely innovations). Prints the values
# for KF.h / SENSORS.h (the current ones are read from there) and plots both.
# Hand moves have no motor vibration: in flight the IMU is worse (accNoise, biasWalk only larger).

import binascii
import math
import os
import re
import socket
import struct
import sys
import time
import numpy as np

PORT = 4210
PKT_LOG, PKT_FLIGHT_CMD, PKT_CAL_IMU, PKT_CAL_TOF, PKT_CAL_CAM = 2, 11, 19, 20, 21
CMD_PING, CMD_CALIBRATE = 6, 7
IMU = struct.Struct("<I10f")            # msg::CalImuSample: t, acc g, gyro rad/s, world acc m/s^2, yaw
TOF = struct.Struct("<I3f2Bfb2f")       # msg::CalTof: t, range, height, signal, valid, status, std, result, innov, innovStd
CAM = struct.Struct("<IBH8fb4f")        # msg::CalCam: t, valid, inliers, vx, vy, rmsPx, fx, pairS, range, rot, std,
                                        #              result, innovX, innovY, innovStdX, innovStdY
GRAVITY = 9.80665

PHASES = [  # name, seconds, what to do
    ("still", 60, "on a stand ~20 cm above a textured floor, camera looking down. Hands off."),
    ("slow", 60, "pick it up: slow moves forward / back, left / right, up / down (10-40 cm), keep it level"),
    ("fast", 30, "fast: quick moves and stops, tilts, turns"),
    ("end", 20, "back on the stand. Hands off."),
]

HERE = os.path.dirname(os.path.abspath(__file__))
M4 = os.path.join(HERE, "..", "..")


def config(header, name):               # the float / int values of a config struct, as they are now
    src = open(os.path.join(M4, header)).read()
    body = src[src.index("struct " + name + " {"):]
    body = body[:body.index("\n};")]
    return {k: float(v) for k, v in re.findall(r"\b(?:float|int)\s+(\w+)\s*=\s*([-+\d.eE]+)f?\s*;", body)}


H = config("FILTERS/headers/KF.h", "HorizontalConfig")
V = config("FILTERS/headers/KF.h", "VerticalConfig")
S = config("SENSORS/headers/SENSORS.h", "SensorsConfig")
MAXDT = config("SENSORS/headers/IMU.h", "ImuConfig")["maxDtS"]


def packet(ptype, payload):
    head = struct.pack("<BI", ptype, len(payload))
    return b"\xA5\x5A" + head + payload + struct.pack("<H", binascii.crc_hqx(head + payload, 0xFFFF))


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


# ---------------------------------------------------------------- recording

class Link:
    def __init__(self, ip):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.setblocking(False)
        self.addr = (ip, PORT)
        self.rx = bytearray()
        self.imu, self.tof, self.cam, self.logs = [], [], [], []
        self.pingT = 0.0

    def cmd(self, c):
        self.sock.sendto(packet(PKT_FLIGHT_CMD, bytes([c])), self.addr)

    def pump(self):
        if time.time() - self.pingT > 0.2:   # the M7 answers whoever sent last
            self.pingT = time.time()
            self.cmd(CMD_PING)
        while True:
            try:
                self.rx += self.sock.recvfrom(4096)[0]
            except OSError:
                break
        for ptype, p in parse(self.rx):
            if ptype == PKT_CAL_IMU:
                self.imu += [IMU.unpack_from(p, k) for k in range(0, len(p) - IMU.size + 1, IMU.size)]
            elif ptype == PKT_CAL_TOF and len(p) == TOF.size:
                self.tof.append(TOF.unpack(p))
            elif ptype == PKT_CAL_CAM and len(p) == CAM.size:
                self.cam.append(CAM.unpack(p))
            elif ptype == PKT_LOG:
                self.logs.append(p.decode(errors="replace").strip())   # not printed: only the instructions on screen

    def nowUs(self):
        return self.imu[-1][0] if self.imu else 0


def key():
    import msvcrt
    return msvcrt.getwch().lower() if msvcrt.kbhit() else ""


def wait(link, keys):
    while True:
        link.pump()
        k = key()
        if k and k in keys:
            return k
        time.sleep(0.01)


def record(ip):
    link = Link(ip)
    print("waiting for the drone (M4 kfCalib, M7 m7Runner) ...")
    while not link.tof:
        link.pump()
        time.sleep(0.01)
    while True:
        print("\nPut it on the stand (~20 cm above a textured floor, camera down), hands off. Enter = calibrate, Esc = quit")
        if wait(link, "\r\x1b") == "\x1b":
            return None
        n = len(link.logs)
        link.cmd(CMD_CALIBRATE)
        t = time.time()
        while time.time() - t < 6 and not any("calibrat" in l and "keep" not in l for l in link.logs[n:]):
            link.pump()
            time.sleep(0.01)
        if any(l.endswith("calibrated") for l in link.logs[n:]):
            print("calibrated")
            break
        print("moved during calibration (or no answer), again")
    link.imu.clear(), link.tof.clear(), link.cam.clear()   # the filters start over here

    phases = []
    for name, secs, what in PHASES:
        print(f"\n[{name}, {secs} s] {what}\n  Enter = start, S = skip, Esc = finish")
        k = wait(link, "\rs\x1b")
        if k == "\x1b":
            break
        if k == "s":
            continue
        t0, end = link.nowUs(), time.time() + secs
        while time.time() < end and key() != "\r":
            link.pump()
            print(f"\r  {name}: {end - time.time():3.0f} s left (Enter = stop)   imu {len(link.imu)}  tof {len(link.tof)}"
                  f"  camera {sum(c[1] for c in link.cam)}   ", end="", flush=True)
            time.sleep(0.02)
        phases.append((name, t0, link.nowUs()))
        print("\a\n  STOP")

    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", time.strftime("kf_%Y%m%d_%H%M%S.npz"))
    np.savez(path, imu=np.array(link.imu, float).reshape(-1, 11), tof=np.array(link.tof, float).reshape(-1, 10),
             cam=np.array(link.cam, float).reshape(-1, 16), phase_names=np.array([p[0] for p in phases]),
             phase_t=np.array([p[1:] for p in phases], float).reshape(-1, 2))
    print("saved " + path)
    return path


# ---------------------------------------------------------------- data

def load(path):
    d = np.load(path)
    imu, tof, cam = d["imu"].copy(), d["tof"].copy(), d["cam"].copy()
    t0 = imu[0, 0]
    sec = lambda us: ((np.asarray(us) - t0 + 2**31) % 2**32 - 2**31) * 1e-6   # M4 clock -> s since the first sample
    imu[:, 0], tof[:, 0], cam[:, 0] = sec(imu[:, 0]), sec(tof[:, 0]), sec(cam[:, 0])
    imu = imu[np.unique(imu[:, 0], return_index=True)[1]]
    tof = tof[np.argsort(tof[:, 0], kind="stable")]
    cam = cam[np.argsort(cam[:, 0], kind="stable")]
    phases = [(str(n), *sec(t)) for n, t in zip(d["phase_names"], d["phase_t"])]
    return imu, tof, cam, phases


def inPhase(t, phases, names, margin=0.0):
    m = np.zeros(len(t), bool)
    for n, a, b in phases:
        if n in names:
            m |= (t > a + margin) & (t < b - margin)
    return m


def allan(x, dt, taus):                 # overlapping Allan deviation
    th = np.concatenate([[0.0], np.cumsum(x) * dt])
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
        out.append(math.sqrt(np.mean(d * d) / 2) / (m * dt))
    return np.array(out)


def still(imu, tof, cam, phases):
    x = imu[inPhase(imu[:, 0], phases, {"still"}, 2.0)]
    if len(x) < 4000:
        print("\nno still phase (or under 10 s): skipped")
        return None
    dt = np.median(np.diff(x[:, 0]))
    gaps = np.sum(np.diff(x[:, 0]) > 2.5 * dt)
    taus = np.unique(np.round(np.logspace(np.log10(dt), np.log10((x[-1, 0] - x[0, 0]) / 4), 40) / dt)) * dt
    print(f"\nSTILL  {x[-1, 0] - x[0, 0]:.0f} s, {1 / dt:.0f} Hz, {gaps} gaps (lost packets)")
    res = {"taus": taus}
    for name, data, unit in (("acc", x[:, 1:4] * GRAVITY, "m/s^2"), ("gyro", x[:, 4:7], "rad/s")):
        ad = np.array([allan(data[:, k] - data[:, k].mean(), dt, taus) for k in range(3)])
        white = np.median((ad * np.sqrt(taus))[:, (taus >= 0.05) & (taus <= 0.5)], axis=1)
        res[name], res[name + "White"] = ad, white
        print(f"  {name:5} std {' '.join(f'{v:.3g}' for v in data.std(0))} {unit}   white noise "
              f"{' '.join(f'{v:.3g}' for v in white)} {unit}/sqrt(Hz)   bias instability "
              f"{' '.join(f'{v:.3g}' for v in ad.min(1) / 0.664)} {unit}")
    t = tof[inPhase(tof[:, 0], phases, {"still"}, 2.0) & (tof[:, 4] > 0)]
    if len(t) > 20:
        res["tofStd"], res["tofRange"] = t[:, 2].std(), t[:, 1].mean()
        print(f"  ToF   height std {t[:, 2].std() * 1e3:.2f} mm at {t[:, 1].mean():.3f} m   (now given: "
              f"{(S['tofStd'] + S['tofStdRange'] * t[:, 1].mean()) * 1e3:.2f} mm)")
    c = cam[inPhase(cam[:, 0], phases, {"still"}, 2.0)]
    v = c[c[:, 1] > 0]
    print(f"  camera  {len(v)} of {len(c)} frames gave a velocity", end="")
    if len(v) > 20:
        res["camStd"] = v[:, 3:5].std(0)
        print(f", std x {v[:, 3].std() * 1e3:.1f} y {v[:, 4].std() * 1e3:.1f} mm/s   (now given: {np.median(v[:, 10]) * 1e3:.1f} mm/s)")
    else:
        print("  (camera too close to the floor? needs ~20 cm)")
    return res


# ---------------------------------------------------------------- filters again, offline
# The same models as KF.cpp. Between two measurements the IMU steps are summed once (they do not
# depend on the noise values): x = F x + u, and the process noise is
#   Q = accNoise^2 * sum_k c_k A_k + biasWalk^2 * B
# where k^2 = (1 + rate / kRateRadS + dev / kAccG)^2 is split into 6 terms (A_k) with the
# coefficients c_k. One fit step is then a few small matrix products per measurement.

def coeffs(kR, kA):
    return np.array([1, 2 / kR, 2 / kA, 1 / kR**2, 1 / kA**2, 2 / (kR * kA)])


def vModel(s, dt):
    h, a = 0.5 * dt * dt, s[9]
    F = np.array([[1, dt, -h], [0, 1, -dt], [0, 0, 1.0]])
    G = np.array([[dt**3 / 3, dt * dt / 2, 0], [dt * dt / 2, dt, 0], [0, 0, 0]])
    return F, np.array([a * h, a * dt, 0]), G, np.diag([0, 0, dt])


def hModel(s, dt):
    c, n, h = math.cos(s[10]), math.sin(s[10]), 0.5 * dt * dt
    F = np.eye(6)
    F[0, 2] = F[1, 3] = dt
    F[0, 4], F[0, 5], F[1, 4], F[1, 5] = -c * h, n * h, -n * h, -c * h
    F[2, 4], F[2, 5], F[3, 4], F[3, 5] = -c * dt, n * dt, -n * dt, -c * dt
    b = np.array([s[7] * h, s[8] * h, s[7] * dt, s[8] * dt, 0, 0])
    G = np.zeros((6, 6))
    for p, v in ((0, 2), (1, 3)):
        G[p, p], G[p, v], G[v, p], G[v, v] = dt**3 / 3, dt * dt / 2, dt * dt / 2, dt
    return F, b, G, np.diag([0, 0, 0, 0, dt, dt])


def prepare(imu, tm, n, model):
    t = imu[:, 0]
    r = np.linalg.norm(imu[:, 4:7], axis=1)
    d = np.abs(np.linalg.norm(imu[:, 1:4], axis=1) - 1)
    basis = np.stack([np.ones_like(r), r, d, r * r, d * d, r * d], 1)
    Fs, us, As, Bs = np.empty((len(tm), n, n)), np.empty((len(tm), n)), np.empty((len(tm), 6, n, n)), np.empty((len(tm), n, n))
    i = 1
    for j, tj in enumerate(tm):
        F, u, A, B = np.eye(n), np.zeros(n), np.zeros((6, n, n)), np.zeros((n, n))
        while i < len(t) and t[i] <= tj:
            dt = min(t[i] - t[i - 1], MAXDT)
            if dt > 0:
                Fi, bi, Gi, Di = model(imu[i], dt)
                F, u = Fi @ F, Fi @ u + bi
                A = Fi @ A @ Fi.T + basis[i][:, None, None] * Gi
                B = Fi @ B @ Fi.T + Di
            i += 1
        Fs[j], us[j], As[j], Bs[j] = F, u, A, B
    return Fs, us, As, Bs


def runV(pre, m, p, keep=False):
    acc, walk, s0, sR, kR, kA = p
    Fs, us, As, Bs = pre
    Q = acc * acc * np.einsum("k,mkab->mab", coeffs(kR, kA), As) + walk * walk * Bs
    R = (s0 + sR * m["range"]) ** 2
    g2 = V["gate"] ** 2
    x, P = np.array([m["z"][0], 0, 0.0]), np.diag([1e-4, 1e-4, V["biasInit"] ** 2])
    seen, rej, cost = m["t"][0], 0, 0.0
    out = np.zeros((len(R), 4))         # innovation, its variance, result, z
    for j in range(len(R)):
        F = Fs[j]
        x, P = F @ x + us[j], F @ P @ F.T + Q[j]
        z, y, Sv = m["z"][j], 0.0, R[j]
        if m["t"][j] - seen > S["tofLostS"]:    # back after long: taken as it is
            res = 3
            x[0], P[0, :], P[:, 0] = z, 0, 0
            P[0, 0] = R[j]
        else:
            y, Sv = z - x[0], P[0, 0] + R[j]
            d2 = y * y / Sv
            cost += math.log(Sv) + min(d2, g2)
            if d2 > g2:
                rej, res = rej + 1, 0
                if rej >= V["forceAfter"]:      # a floor step
                    x, P, rej, res = np.array([z, 0, x[2]]), np.diag([R[j], V["stepVelStd"] ** 2, V["biasInit"] ** 2]), 0, 2
            else:
                K = P[:, 0] / Sv
                x, P, rej, res = x + K * y, P - np.outer(K, P[0]), 0, 1
        seen = m["t"][j]
        if keep:
            out[j] = y, Sv, res, x[0]
    return cost, out


def runH(pre, m, p, keep=False):
    acc, walk, floor, scale, rot, kR, kA = p
    Fs, us, As, Bs = pre
    Q = acc * acc * np.einsum("k,mkab->mab", coeffs(kR, kA), As) + walk * walk * Bs
    R = m["pp2"] + floor**2 + (scale * m["speed"]) ** 2 + (rot * m["rotv"]) ** 2
    g2 = H["gate"] ** 2 * 2
    x, P = np.zeros(6), np.diag([1e-6, 1e-6, 1e-4, 1e-4, H["biasInit"] ** 2, H["biasInit"] ** 2])
    seen, rej, cost = m["t"][0], 0, 0.0
    out = np.zeros((len(R), 8))         # innovation x y, variance x y, d^2, result, vx, vy

    def setV(z, r):                     # setVelocity: the velocity IS z
        x[2:4] = z
        P[2:4, :], P[:, 2:4] = 0, 0
        P[2, 2] = P[3, 3] = r

    for j in range(len(R)):
        F = Fs[j]
        x[:], P[:] = F @ x + us[j], F @ P @ F.T + Q[j]
        z, y, Sm, d2 = m["v"][j], np.zeros(2), np.diag([R[j], R[j]]), 0.0
        if m["t"][j] - seen > S["camLostS"]:
            res = 3
            setV(z, R[j])
        else:
            y, Sm = z - x[2:4], P[2:4, 2:4] + np.diag([R[j], R[j]])
            det = Sm[0, 0] * Sm[1, 1] - Sm[0, 1] * Sm[1, 0]
            Si = np.array([[Sm[1, 1], -Sm[0, 1]], [-Sm[1, 0], Sm[0, 0]]]) / det
            d2 = y @ Si @ y
            cost += math.log(det) + min(d2, g2)
            if d2 > g2:
                rej, res = rej + 1, 0
                if rej >= H["forceAfter"]:
                    setV(z, R[j])
                    rej, res = 0, 4
            else:
                K = P[:, 2:4] @ Si
                x[:], P[:], rej, res = x + K @ y, P - K @ P[2:4, :], 0, 2
        seen = m["t"][j]
        if keep:
            out[j] = y[0], y[1], Sm[0, 0], Sm[1, 1], d2, res, x[2], x[3]
    return cost, out


def fit(run, pre, m, p0, free):
    from scipy.optimize import minimize
    p0 = np.array(p0, float)

    def cost(lx):
        p = p0.copy()
        p[free] = np.exp(np.clip(lx, -14, 3))
        return run(pre, m, p)[0]

    r = minimize(cost, np.log(p0[free]), method="Nelder-Mead",
                 options={"maxfev": 120 * len(free), "xatol": 0.02, "fatol": 0.05})
    p = p0.copy()
    p[free] = np.exp(np.clip(r.x, -14, 3))
    return p


def report(title, t, d2, res, dims, phases):
    print(f"  {title:8}", end="")
    for name in [p[0] for p in phases] + ["all"]:
        mk = (res != 3) & (inPhase(t, phases, {name}) if name != "all" else True)
        if mk.sum():
            print(f"  {name} NIS {np.mean(np.minimum(d2[mk], 25 * dims)) / dims:5.2f} refused {np.mean(res[mk] == 0) * 100:4.1f}%", end="")
    print()


def line(name, now, new, note=""):
    if new < now / 10:
        note += "   barely seen in this recording: keep the old one?"
    print(f"    float {name:<12}= {new:.3g}f;   // was {now:g}{note}")


def analyse(path):
    imu, tof, cam, phases = load(path)
    print(f"{path}: {imu[-1, 0]:.0f} s, imu {len(imu)}, tof {len(tof)}, camera {len(cam)} ("
          + ", ".join(f"{n} {b - a:.0f} s" for n, a, b in phases) + ")")
    st = still(imu, tof, cam, phases)

    tv = tof[(tof[:, 7] != -2) & (tof[:, 0] > imu[0, 0])]
    mv = {"t": tv[:, 0], "z": tv[:, 2], "range": tv[:, 1]}
    tc = cam[(cam[:, 1] > 0) & (cam[:, 11] != -2) & (cam[:, 0] > imu[0, 0])]
    pp = np.maximum(tc[:, 5], S["camMinPx"]) * tc[:, 8] / (tc[:, 6] * tc[:, 7])
    mh = {"t": tc[:, 0], "v": tc[:, 3:5], "pp2": pp * pp / np.maximum(tc[:, 2], 1),
          "speed": np.hypot(tc[:, 3], tc[:, 4]), "rotv": tc[:, 9] * tc[:, 8] / tc[:, 7]}
    k0 = [S["kRateRadS"], S["kAccG"]]

    print("\nREPLAY  the filters again on the recording; NIS ~1 = the noise values fit (>1: too small, <1: too big)")
    hp = None
    if len(tc) >= 100:
        given = np.sqrt(mh["pp2"] + S["camStdFloor"] ** 2 + (S["camStdScale"] * mh["speed"]) ** 2 + (S["camStdRot"] * mh["rotv"]) ** 2)
        print(f"  camera std rebuilt vs the drone's: {np.median(np.abs(given / tc[:, 10] - 1)) * 100:.1f}% off (should be ~0)")
        print("  horizontal: preparing ...", flush=True)
        preH = prepare(imu, mh["t"], 6, hModel)
        h0 = [H["accNoise"], H["biasWalk"], S["camStdFloor"], S["camStdScale"], S["camStdRot"]] + k0
        _, oh0 = runH(preH, mh, h0, True)
        ok = tc[:, 11] > 0
        print(f"  replay vs drone, camera innovation: {np.median(np.abs(oh0[ok, 0] - tc[ok, 12])) * 1e3:.1f} mm/s apart (median)")
        print("  horizontal: fitting ...", flush=True)
        hp = fit(runH, preH, mh, h0, [0, 1, 2, 3, 4, 5, 6])
        _, oh = runH(preH, mh, hp, True)
        k0 = list(hp[5:7])
        report("now", mh["t"], oh0[:, 4], oh0[:, 5], 2, phases)
        report("fitted", mh["t"], oh[:, 4], oh[:, 5], 2, phases)
    else:
        print(f"  horizontal: only {len(tc)} camera velocities, not fitted")

    vp = None
    if len(tv) >= 100:
        print("  vertical: preparing ...", flush=True)
        preV = prepare(imu, mv["t"], 3, vModel)
        v0 = [V["accNoise"], V["biasWalk"], S["tofStd"], S["tofStdRange"]] + k0
        _, ov0 = runV(preV, mv, v0, True)
        ok = tv[:, 7] == 1
        print(f"  replay vs drone, ToF innovation: {np.median(np.abs(ov0[ok, 0] - tv[ok, 8])) * 1e3:.2f} mm apart (median)")
        print("  vertical: fitting ...", flush=True)
        vp = fit(runV, preV, mv, v0, [0, 1, 2, 3] if hp is not None else [0, 1, 2, 3, 4, 5])
        _, ov = runV(preV, mv, vp, True)
        report("now", mv["t"], ov0[:, 0] ** 2 / ov0[:, 1], ov0[:, 2], 1, phases)
        report("fitted", mv["t"], ov[:, 0] ** 2 / ov[:, 1], ov[:, 2], 1, phases)
    else:
        print(f"  vertical: only {len(tv)} ToF readings, not fitted")

    print("\nSUGGESTED (hand moves: in flight vibration needs accNoise / biasWalk at least this, usually more)")
    if hp is not None:
        print("  M4/FILTERS/headers/KF.h  HorizontalConfig")
        line("accNoise", H["accNoise"], hp[0])
        line("biasWalk", H["biasWalk"], hp[1])
    if vp is not None:
        print("  M4/FILTERS/headers/KF.h  VerticalConfig")
        line("accNoise", V["accNoise"], vp[0])
        line("biasWalk", V["biasWalk"], vp[1], "   keep >= 0.1 for flight (vibration, flight logs 31.10)")
    if hp is not None or vp is not None:
        print("  M4/SENSORS/headers/SENSORS.h  SensorsConfig")
    if hp is not None:
        line("camStdFloor", S["camStdFloor"], hp[2])
        line("camStdScale", S["camStdScale"], hp[3])
        line("camStdRot", S["camStdRot"], hp[4])
    if vp is not None:
        line("tofStd", S["tofStd"], vp[2])
        line("tofStdRange", S["tofStdRange"], vp[3])
    k = hp[5:7] if hp is not None else (vp[4:6] if vp is not None else None)
    if k is not None:
        line("kRateRadS", S["kRateRadS"], k[0])
        line("kAccG", S["kAccG"], k[1])
    if st is not None and "accWhite" in st:
        print(f"  (the accelerometer alone is {st['accWhite'].mean():.3g} m/s^2/sqrt(Hz): accNoise never below that)")

    import matplotlib.pyplot as plt

    def shade(ax):
        for n, a, b in phases:
            ax.axvspan(a, b, alpha=0.08, color="C0" if n in ("still", "end") else "C3")
            ax.text(a, 1, n, transform=ax.get_xaxis_transform(), va="top", fontsize=8)

    if st is not None:
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
        for a, name, unit in ((ax[0], "acc", "m/s^2"), (ax[1], "gyro", "rad/s")):
            for k2, c in enumerate("xyz"):
                a.loglog(st["taus"], st[name][k2], label=c)
            a.set_xlabel("tau, s")
            a.set_ylabel(f"Allan deviation, {unit}")
            a.set_title(name + " (still)")
            a.grid(True, which="both", alpha=0.3)
            a.legend()
        fig.tight_layout()
    if vp is not None:
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
        ax[0].plot(mv["t"], mv["z"], ".", ms=2, label="ToF height")
        ax[0].plot(mv["t"], ov[:, 3], label="filter (fitted)")
        ax[0].set_ylabel("z, m")
        ax[0].legend()
        ax[1].plot(mv["t"], ov0[:, 0] / np.sqrt(ov0[:, 1]), ".", ms=2, label="now")
        ax[1].plot(mv["t"], ov[:, 0] / np.sqrt(ov[:, 1]), ".", ms=2, label="fitted")
        ax[1].set_ylabel("ToF innovation / std")
        ax[1].set_ylim(-8, 8)
        ax[1].legend()
        ax[1].set_xlabel("s")
        for a in ax:
            shade(a)
        fig.suptitle("vertical filter")
        fig.tight_layout()
    if hp is not None:
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 9))
        for a, k2, c in ((ax[0], 0, "vx"), (ax[1], 1, "vy")):
            a.plot(mh["t"], mh["v"][:, k2], ".", ms=2, label="camera")
            a.plot(mh["t"], oh[:, 6 + k2], label="filter (fitted)")
            a.set_ylabel(c + ", m/s")
            a.legend()
        ax[2].plot(mh["t"], np.sqrt(oh0[:, 4] / 2), ".", ms=2, label="now")
        ax[2].plot(mh["t"], np.sqrt(oh[:, 4] / 2), ".", ms=2, label="fitted")
        ax[2].axhline(1, color="k", lw=0.5)
        ax[2].set_ylabel("camera innovation / std")
        ax[2].set_ylim(0, 6)
        ax[2].legend()
        ax[2].set_xlabel("s")
        for a in ax:
            shade(a)
        fig.suptitle("horizontal filter")
        fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--fit":
        analyse(sys.argv[2])
    elif len(sys.argv) == 2:
        p = record(sys.argv[1])
        if p:
            analyse(p)
    else:
        print("python kfCalib.py NICLA_IP  |  python kfCalib.py --fit FILE.npz")
