# makeDataset - every airborne moment of every flight log, in one CSV, with labels for learning what the
# throttle model misses (Neural-Lander style). Run again after new flights:
#   python makeDataset.py
#
# Reads:  collectedData/oldLogs/hoverTest/hover_*.csv    (the old hoverTest.py / autoTune.py, 17.11)
#         controlApp/logs/session_*_droneN.csv           (controlApp)
# Writes: flights.csv    all airborne rows, all firmware versions (filter with era / useForTraining)
#         training_z.csv only the rows to train on, only the columns a z model needs
#
# Airborne = armed, throttle > 0.1 and height > 5 cm: from the data, not from `state` (its numbers changed:
# BOOT READY TAKEOFF FLY ... until early November, BOOT READY SPINUP CLIMB FLY SAFETY LAND from 17.11).
#
# Labels (z, world, m/s^2):
#   aTrueZ    the real vertical acceleration: the height smoothed forwards AND backwards (Savitzky-Golay,
#             SMOOTH_S window, cubic) and differentiated twice. Offline only: it looks into the future.
#   aModelZ   what the throttle model expected, the controller's own model (PosCTRL / StateMachine):
#             f1 (T(t - THRUST_DELAY_S) cos(roll) cos(pitch) - Th(V)) - f2 vz,  Th = HOVER_V / battery,  f1 = 27.5, f2 = 0.5
#   residualZ aTrueZ - aModelZ: what a network would learn and the controller would subtract from aCmd.
# The same smoothing gives aTrueX / aTrueY (world) for later x / y models.
#
# Pushes (a hand, a knock): the drone tilts without being told to, or is accelerated in a way the commands do
# not explain. Flagged when, for at least PUSH_MIN samples close together:
#   attErr    |measured - commanded roll / pitch| (each segment's own offset removed) > PUSH_ATT_DEG
#   |aTrue x/y| > PUSH_AXY,   or |residualZ| > PUSH_RZ above PUSH_MIN_Z (a landing touch is not a push)
# pushed = 1 for the event +- PUSH_PAD_S; never used for training. Every event: pushes.csv.
#
# Corrupted parts, never used for training either:
#   attDrift  the drone's roll / pitch estimate drifting away (gyro alone, no Betaflight reference: log 23.11
#             21:58 reached -18 / +25 deg level). Measured - commanded, averaged over DRIFT_AVG_S, against the
#             segment's first DRIFT_REF_S; above DRIFT_DEG the tilt (and so aModelZ) is wrong.
#   height    ToF invalid or the estimate off the ToF (a jump of the filter): those rows AND +- SMOOTH_S / 2
#             around them, the smoothing for aTrue reaches that far.
#   battery   below MIN_BATTERY_V (smoothed).
#   offset    the flight's attitude offset (first DRIFT_REF_S) above ATT_OFFSET_DEG: the whole flight.
#   manual    MANUAL_CUTS: what only the pilot knows (a hand, a punch).

import csv
import glob
import math
import os

import numpy as np
from scipy.signal import savgol_filter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SOURCES = sorted(glob.glob(os.path.join(HERE, "oldLogs", "hoverTest", "hover_*.csv"))) + \
          sorted(glob.glob(os.path.join(ROOT, "controlApp", "logs", "session_*_drone*.csv")))
SOURCES = [f for f in SOURCES if not f.endswith(("_flow.csv", "_points.csv", "_depth.csv"))]

FIELDS = ["tUs", "state", "flags", "x", "y", "z", "yaw", "vx", "vy", "vz", "vyaw", "tx", "ty", "tz", "tyaw",
          "roll", "pitch", "yawRate", "throttle", "ix", "iy", "iz", "ax", "ay", "az", "attRoll", "attPitch", "battery",
          "tofRange", "tofHeight", "tofSignal", "tofValid", "tofStatus", "floorSteps", "camYawRate", "gyroYawRate"]
STATES_C = ["BOOT", "READY", "SPINUP", "CLIMB", "FLY", "SAFETY", "LAND"]   # era C only
ARMED = 1
HOVER_V, F1, F2, G = 2.59, 27.5, 0.5, 9.81   # F1: measured 24.11 (was 60, as PosCTRL f1.z then)
THRUST_DELAY_S = 0.12                   # the thrust follows the throttle this late (MSP, Betaflight smoothing, motors,
                                        # height-estimate lag): best fit on 29 min (24.11), residual rms 0.61 -> 0.51 m/s^2.
                                        # Without it the network learned the delay through vz (R2 0.56, 90 % of it vz)
SMOOTH_S = 0.6                          # smoothing window for the true acceleration
MAX_GAP_S = 0.06                        # a longer gap in the telemetry starts a new segment
MIN_SEG_S = 1.5                         # shorter segments get no labels
CRASH_ACC = 15.0                        # |aTrue| above this: a hit, not flying
PUSH_ATT_DEG, PUSH_AXY, PUSH_RZ = 10.0, 2.5, 4.0   # normal flight (era C): 99 % below 7.8 deg, 1.4, 3.3 m/s^2
PUSH_MIN_Z, PUSH_MIN, PUSH_JOIN_S, PUSH_PAD_S = 0.15, 3, 1.5, 1.0
DRIFT_DEG, DRIFT_AVG_S, DRIFT_REF_S = 3.0, 5.0, 10.0   # good flights (23.11 17:57): < 1 deg
TOF_OFF_M = 0.03                        # |z - ToF height| above this: the height filter jumped
FLIGHT_GAP_S = 5.0                      # segments closer than this: the same flight (one drift offset)
ATT_OFFSET_DEG = 4.0                    # |measured - commanded| over a flight's first DRIFT_REF_S above this: the
                                        # Betaflight offset was taken wrong, the whole flight out (normal: < 2 deg;
                                        # 24.11 10:14: +7.5 pitch, our pitch ran to +7 during the climb)
MANUAL_CUTS = {                         # file: [(from, to)] s since the log's first row; things only the pilot knows
    "session_20261124_104923_drone0.csv": [(198.0, math.inf)],   # punched by hand at 198.4 s, then killed
}
MIN_BATTERY_V = 6.6                    # 2S below 3.3 V / cell: the voltage collapses, thrust no longer ~ throttle / V
                                        # (log 23.11 22:28: 6.7 -> 5.0 V in 15 s, then it fell)


def era(name):                          # firmware generation, from the file's date
    d = name.split("_")[1]
    return "A" if d < "20261103" else ("B" if d < "20261117" else "C")


def read(path):
    with open(path) as f:
        r = csv.reader(f)
        head = next(r, None)
        if not head:
            return None
        idx = {k: i for i, k in enumerate(head)}
        rows = []
        for row in r:
            try:
                rows.append([float(row[idx[k]]) if k in idx and row[idx[k]] != "" else math.nan for k in FIELDS])
            except (ValueError, IndexError):
                pass
    return np.array(rows) if rows else None


def segments(t, air):                   # index ranges: airborne, no gap longer than MAX_GAP_S
    out, start = [], None
    for i in range(len(t)):
        ok = air[i] and (start is None or 0 < t[i] - t[i - 1] <= MAX_GAP_S)   # back in time: the drone rebooted
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            out.append((start, i))
            start = i if air[i] else None
    if start is not None:
        out.append((start, len(t)))
    return out


def smooth(a, n):                       # centred rolling mean, n samples
    k = np.ones(n) / n
    p = np.pad(a, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(p, k, mode="valid")


def grow(mask, n):                      # every True spread n samples to both sides
    return np.convolve(mask.astype(float), np.ones(2 * n + 1), mode="same") > 0


def drift(ts, er, ep, dt, ref):         # deg: how far the averaged attitude error moved from the flight's start
    n = max(1, int(round(DRIFT_AVG_S / dt)))   # ref: (roll, pitch) offset of this flight, None: from here
    mr, mp = smooth(np.nan_to_num(er), n), smooth(np.nan_to_num(ep), n)
    if ref is None:
        k = ts <= DRIFT_REF_S
        ref = (float(np.median(mr[k])), float(np.median(mp[k])))
    return np.hypot(mr - ref[0], mp - ref[1]), ref


OUT_COLS = ["file", "era", "segment", "t", "stateName"] + FIELDS + \
           ["batterySmooth", "hoverThrottle", "tiltCos", "aTrueX", "aTrueY", "aTrueZ", "aModelZ", "residualZ",
            "attErr", "attDrift", "heightBad", "crash", "pushed", "useForTraining"]
TRAIN_COLS = ["file", "segment", "t", "z", "vx", "vy", "vz", "attRoll", "attPitch", "throttle", "batterySmooth", "tofSignal",
              "aModelZ", "aTrueZ", "residualZ"]

def pushes(ts, attErr, axy, rz, z):     # -> (pushed mask, [(i0, t0, t1, peak att, peak axy, peak rz, reason)])
    hT, hS, hV = attErr > PUSH_ATT_DEG, axy > PUSH_AXY, (rz > PUSH_RZ) & (z > PUSH_MIN_Z)
    hit = hT | hS | hV
    mask, events, groups, cur = np.zeros(len(ts), bool), [], [], []
    for i in np.nonzero(hit)[0]:        # hits closer than PUSH_JOIN_S belong to one event
        if cur and ts[i] - ts[cur[-1]] > PUSH_JOIN_S:
            groups.append(cur)
            cur = []
        cur.append(i)
    if cur:
        groups.append(cur)
    for gr in groups:
        if len(gr) < PUSH_MIN:
            continue
        t0, t1 = ts[gr[0]] - PUSH_PAD_S, ts[gr[-1]] + PUSH_PAD_S
        mask |= (ts >= t0) & (ts <= t1)
        reason = ("T" if hT[gr].any() else "") + ("S" if hS[gr].any() else "") + ("V" if hV[gr].any() else "")
        events.append((int(np.searchsorted(ts, t0)), max(t0, 0.0), min(t1, ts[-1]), np.nanmax(attErr[gr]),
                       np.nanmax(axy[gr]), np.nanmax(rz[gr]), reason))
    return mask, events


seg_id, n_all, n_train, minutes = 0, 0, 0, {"A": 0.0, "B": 0.0, "C": 0.0}
n_push, push_rows = 0, []
drop = {"offset": 0, "manual": 0, "drift": 0, "height": 0, "battery": 0}   # era C rows left out (not crash / push)
with open(os.path.join(HERE, "flights.csv"), "w", newline="") as fa, \
     open(os.path.join(HERE, "training_z.csv"), "w", newline="") as ft:
    wa, wt = csv.writer(fa), csv.writer(ft)
    wa.writerow(OUT_COLS)
    wt.writerow(TRAIN_COLS)
    for path in SOURCES:
        d = read(path)
        if d is None or len(d) < 10:
            continue
        name, e = os.path.basename(path), era(os.path.basename(path))
        col = {k: d[:, i] for i, k in enumerate(FIELDS)}
        t = np.unwrap(col["tUs"] * 1e-6, period=2 ** 32 * 1e-6)     # the shared clock wraps every 71 min
        armed = (np.nan_to_num(col["flags"]).astype(int) & ARMED) > 0
        air = armed & (col["throttle"] > 0.1) & (col["z"] > 0.05)
        ref, tEnd = None, -math.inf          # attitude offset of the flight, end of the last segment
        for s0, s1 in segments(t, air):
            if t[s1 - 1] - t[s0] < MIN_SEG_S:
                continue
            seg_id += 1
            ts = t[s0:s1] - t[s0]
            dt = float(np.median(np.diff(ts)))
            win = max(5, int(round(SMOOTH_S / dt)) | 1)
            c = {k: v[s0:s1] for k, v in col.items()}
            aT = {ax: savgol_filter(c[ax], win, 3, deriv=2, delta=dt) for ax in ("x", "y", "z")}
            vb = smooth(c["battery"], max(1, int(round(2.0 / dt))))
            th = np.where(vb > 5.0, HOVER_V / np.maximum(vb, 1e-3), np.nan)
            tilt = np.cos(np.radians(c["attRoll"])) * np.cos(np.radians(c["attPitch"]))
            nd = int(round(THRUST_DELAY_S / dt))      # the thrust of THRUST_DELAY_S ago (first nd rows: NaN, unused)
            thrD = np.concatenate([np.full(nd, np.nan), c["throttle"][:len(ts) - nd]]) if nd else c["throttle"]
            aM = F1 * (thrD * tilt - th) - F2 * c["vz"]
            res = aT["z"] - aM
            crash = (np.abs(aT["z"]) > CRASH_ACC) | (np.abs(aT["x"]) > CRASH_ACC) | (np.abs(aT["y"]) > CRASH_ACC)
            er, ep = c["attRoll"] - c["roll"], c["attPitch"] - c["pitch"]
            attErr = np.hypot(er - np.nanmedian(er), ep - np.nanmedian(ep))   # the segment's own offset out
            pushed, events = pushes(ts, attErr, np.hypot(aT["x"], aT["y"]), np.abs(res), c["z"])
            for ev in events:                # line in the original log: header = line 1
                push_rows.append([name, e, seg_id, s0 + ev[0] + 2, f"{ev[1]:.2f}", f"{ev[2]:.2f}", ev[6],
                                  f"{ev[3]:.1f}", f"{ev[4]:.2f}", f"{ev[5]:.2f}"])
            n_push += int(pushed.sum())
            if t[s0] - tEnd > FLIGHT_GAP_S:   # a new flight (not a telemetry gap): new offset
                ref = None
            aDrift, ref = drift(ts, er, ep, dt, ref)
            tEnd = t[s1 - 1]
            hBad = grow((c["tofValid"] != 1) | ~(np.abs(c["z"] - c["tofHeight"]) < TOF_OFF_M), win // 2)
            base = (e == "C") & ~crash & ~pushed & np.isfinite(res)
            lowBat = ~(vb >= MIN_BATTERY_V)
            tLog = t[s0:s1] - t[0]
            cut = np.zeros(s1 - s0, bool)
            for a, b in MANUAL_CUTS.get(name, []):
                cut |= (tLog >= a) & (tLog < b)
            if math.hypot(*ref) > ATT_OFFSET_DEG:     # the whole flight leans on a wrong reference
                drop["offset"] += int(base.sum())
                base = np.zeros_like(base)
            drop["manual"] += int((base & cut).sum())
            base = base & ~cut
            good = base & (aDrift < DRIFT_DEG) & ~hBad & ~lowBat
            drop["drift"] += int((base & (aDrift >= DRIFT_DEG)).sum())
            drop["height"] += int((base & (aDrift < DRIFT_DEG) & hBad).sum())
            drop["battery"] += int((base & (aDrift < DRIFT_DEG) & ~hBad & lowBat).sum())
            minutes[e] += (ts[-1]) / 60
            for i in range(s1 - s0):
                st = int(c["state"][i]) if np.isfinite(c["state"][i]) else -1
                sname = STATES_C[st] if e == "C" and 0 <= st < len(STATES_C) else ""
                raw = ["" if not np.isfinite(c[k][i]) else (int(c[k][i]) if k in ("tUs", "state", "flags", "tofValid", "tofStatus", "floorSteps") else f"{c[k][i]:.5f}") for k in FIELDS]
                der = [f"{vb[i]:.3f}", f"{th[i]:.4f}", f"{tilt[i]:.5f}", f"{aT['x'][i]:.4f}", f"{aT['y'][i]:.4f}",
                       f"{aT['z'][i]:.4f}", f"{aM[i]:.4f}", f"{res[i]:.4f}", f"{attErr[i]:.2f}",
                       f"{aDrift[i]:.2f}", int(hBad[i]), int(crash[i]),
                       int(pushed[i]), int(good[i])]
                wa.writerow([name, e, seg_id, f"{ts[i]:.3f}", sname] + raw + der)
                n_all += 1
                if good[i]:
                    tr = dict(zip(OUT_COLS, [name, e, seg_id, f"{ts[i]:.3f}", sname] + raw + der))
                    wt.writerow([tr[k] for k in TRAIN_COLS])
                    n_train += 1

with open(os.path.join(HERE, "pushes.csv"), "w", newline="") as fp:
    w = csv.writer(fp)
    w.writerow(["file", "era", "segment", "lineInLog", "tStart", "tEnd", "reason", "peakAttErrDeg", "peakAccXY",
                "peakResidualZ"])   # reason: T tilt not commanded, S sideways push, V vertical only (often a height jump)
    w.writerows(push_rows)
print(f"pushes: {len(push_rows)} events, {n_push} rows ({n_push / max(n_all, 1) * 100:.1f} %) -> pushes.csv")
print(f"{len(SOURCES)} logs -> {seg_id} airborne segments, {n_all} rows (flights.csv), {n_train} for training (training_z.csv)")
print("airborne minutes per era:", {k: round(v, 1) for k, v in minutes.items()})
print(f"left out: wrong attitude offset {drop['offset']}, manual cuts {drop['manual']}, attitude drift {drop['drift']}, "
      f"height glitch {drop['height']}, battery < {MIN_BATTERY_V} V {drop['battery']} rows")
