# Every calibrated Nicla -> its measured camera + IMU for the firmware, whoopCTRL/drones/calibration.h.
#   whoopCTRL/drones/<name>_<uid>/calibration.npz  (calib_run.py)  camera
#   whoopCTRL/drones/<name>_<uid>/imu_calib.json   (imu_calib.py)  IMU
# Each folder becomes CAL_<name>; drones/drones.h (by hand) says which drone uses it, with its gains etc.
# A folder without a UID in its name, or without both files, is left out (and said so).
# Generated - don't edit calibration.h by hand, re-run this after every new calibration.
#
# python gen_calib_headers.py
# pip install numpy

import datetime
import hashlib
import json
import os
import re

import numpy as np

import niclaDir

OUT = os.path.join(niclaDir.NICLAS, "calibration.h")
DRONES_H = os.path.join(niclaDir.NICLAS, "drones.h")

# GC2145 modes in the Arduino driver: width -> (height, subsample s, window x0, window y0)
MODES = {160: (120, 4, 480, 360), 320: (240, 3, 320, 240), 640: (480, 2, 160, 120)}


def f(x):
    """C++ float literal with enough digits."""
    x = float(x)
    if not np.isfinite(x):
        raise SystemExit(f"bad value in calibration: {x}")
    s = f"{x:.9g}"
    if "." not in s and "e" not in s:
        s += ".0"                                  # "1f" is not valid C++, "1.0f" is
    return s + "f"


def arr(v):
    return "{" + ", ".join(f(x) for x in np.ravel(v)) + "}"


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:12]


def convert(K, cal_w, new_w):
    """Intrinsics from the calibrated mode to another mode (fallback if the npz lacks K_<w>)."""
    _, s1, x1, y1 = MODES[cal_w]
    _, s2, x2, y2 = MODES[new_w]
    K2 = K.copy()
    K2[0, 0], K2[1, 1] = K[0, 0] * s1 / s2, K[1, 1] * s1 / s2
    K2[0, 2] = (x1 + s1 * K[0, 2] - x2) / s2
    K2[1, 2] = (y1 + s1 * K[1, 2] - y2) / s2
    return K2


def camera_part(path):
    d = np.load(path)
    for key in ("camera_matrix", "dist_coeff"):
        if key not in d.files:
            raise SystemExit(f"{path}: missing '{key}'")
    K = d["camera_matrix"]
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        raise SystemExit(f"{path}: focal length must be positive")
    dist = np.ravel(d["dist_coeff"])
    dist = np.pad(dist, (0, max(0, 5 - len(dist))))[:5]      # k1 k2 p1 p2 k3
    cal_w = int(d["cal_width"]) if "cal_width" in d.files else int(d["image_size"][0])
    cams = []
    for w, (h, *_rest) in sorted(MODES.items()):
        Kw = d[f"K_{w}"] if f"K_{w}" in d.files else convert(K, cal_w, w)
        cams.append(f"{{{f(Kw[0, 0])}, {f(Kw[1, 1])}, {f(Kw[0, 2])}, {f(Kw[1, 2])}, {w}, {h}}}")
    uid = str(d["uid"]) if "uid" in d.files else None
    return cams, dist, uid


def imu_part(path):
    c = json.load(open(path))
    missing = [k for k in ("gyro_bias_dps", "gyro_scale", "accel_bias_g", "accel_scale") if k not in c]
    if missing:
        raise SystemExit(f"{path}: missing {missing}")
    has_fit = "accel_M" in c and "accel_bias_fit_g" in c
    M = np.array(c["accel_M"]) if has_fit else np.eye(3)
    b_fit = np.array(c["accel_bias_fit_g"]) if has_fit else np.zeros(3)
    return c, has_fit, M, b_fit


def ident(name, uid):
    return "CAL_" + re.sub(r"\W", "_", name or uid)


def entry(folder):
    name, uid = niclaDir.parse(os.path.basename(folder))
    cam_p, imu_p = os.path.join(folder, "calibration.npz"), os.path.join(folder, "imu_calib.json")
    cams, dist, cam_uid = camera_part(cam_p)
    c, has_fit, M, b_fit = imu_part(imu_p)
    for what, u in (("calibration.npz", cam_uid), ("imu_calib.json", c.get("uid"))):
        if u is not None and u != uid:            # older files have no UID: then the folder name decides
            raise SystemExit(f"{folder}: {what} is of board {u}, the folder says {uid}")
    rows = ", ".join(arr(r) for r in M)
    return ident(name, uid), "\n".join([
        f"// {os.path.basename(folder)}: calibration.npz {sha(cam_p)}, imu_calib.json {sha(imu_p)}",
        f"// IMU: {c.get('settings', 'unknown')}, {c.get('temp_C', 0)} degC",
        f"constexpr Calibration {ident(name, uid)} = {{\"{uid}\", \"{name or uid}\",",
        f"    {cams[0]}, {cams[1]}, {cams[2]},",
        f"    {arr(dist)},",
        f"    {arr(c['gyro_bias_dps'])}, {arr(c['gyro_scale'])},",
        f"    {'true' if has_fit else 'false'}, {arr(c['accel_bias_g'])}, {arr(c['accel_scale'])},",
        f"    {arr(b_fit)}, {{{rows}}}}};",
    ])


def main():
    entries, names = [], []
    for d in sorted(os.listdir(niclaDir.NICLAS)):
        path = os.path.join(niclaDir.NICLAS, d)
        if not os.path.isdir(path):
            continue
        if niclaDir.parse(d) is None:
            print(f"skipped {d}: no UID in the name (niclaId.ino prints it; rename to {d}_<uid>)")
            continue
        have = [n for n in ("calibration.npz", "imu_calib.json") if os.path.exists(os.path.join(path, n))]
        if len(have) < 2:
            print(f"skipped {d}: has {have or 'nothing'}, needs calibration.npz + imu_calib.json")
            continue
        name, text = entry(path)
        entries.append(text)
        names.append(name)

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    text = "\n".join([
        f"// Generated by test/gen_calib_headers.py on {now} from drones/<name>_<uid>/ - DO NOT EDIT.",
        "// Measured per Nicla; drones.h includes it (the struct is there) and gives each drone its CAL_<name>.",
        "// Camera 160x120 / 320x240 / 640x480 {fx, fy, cx, cy, w, h}, distortion {k1 k2 p1 p2 k3}, gyro bias dps,",
        "// gyro scale, acc has fit, acc bias g, acc scale, acc bias (fit) g, acc M.",
        "",
        "#pragma once",
        "",
        "namespace drones {",
        "",
        "\n\n".join(entries),
        "",
        "}",
        "",
    ])
    with open(OUT, "w", newline="\n") as fh:
        fh.write(text)
    print(f"wrote {OUT}: {names}")
    used = open(DRONES_H, encoding="utf8").read() if os.path.exists(DRONES_H) else ""
    for n in names:
        if n not in used:
            print(f"  {n} is not used in drones/drones.h yet: add a drone with d.cal = &{n} to DRONES[]")


if __name__ == "__main__":
    main()
