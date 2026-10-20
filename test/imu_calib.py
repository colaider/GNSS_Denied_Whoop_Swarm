# Guided IMU (LSM6DSOX) calibration for the Nicla Vision, used with imu_calib.ino.
#   1) gyro bias      - board still for 20 s
#   2) accel 6 faces  - each axis pointing up and down, 5 s each, any order
#   3) accel full fit - ~24 random still poses, ellipsoid fit (bias, scale, cross-axis)
#   4) gyro scale     - optional, exact 360 deg turn about each axis
# Result: imu_calib.json + C++ constants to paste into the flight code.
# pip install pyserial numpy
# python imu_calib.py          (auto-finds the Nicla)
# python imu_calib.py COM5     (or give the port)

import json
import sys
import threading
import time

import numpy as np

GYRO_STILL_DPS = 0.5      # max gyro std while "still"
ACC_STILL_G = 0.01        # max accel std while "still"

# ---- calibration maths (rows: t_s, ax, ay, az, gx, gy, gz, temp) ----


def is_still(d):
    return d[:, 4:7].std(0).max() < GYRO_STILL_DPS and d[:, 1:4].std(0).max() < ACC_STILL_G


def face_name(mean_acc):
    """Which axis points up: at rest the accelerometer reads +1 g on the 'up' axis."""
    i = int(np.argmax(np.abs(mean_acc)))
    return ("+" if mean_acc[i] > 0 else "-") + "XYZ"[i]


def accel_from_faces(faces):
    """Per-axis bias and scale from the 6 faces: a_cal = (a_raw - bias) / scale."""
    bias, scale = np.zeros(3), np.zeros(3)
    for i, a in enumerate("XYZ"):
        up, down = faces["+" + a][i], faces["-" + a][i]
        bias[i] = (up + down) / 2
        scale[i] = (up - down) / 2
    return bias, scale


def ellipsoid_fit(P):
    """Fit a_cal = M @ (a_raw - b) so every still pose has |a_cal| = 1 g.
    M is symmetric (scale + cross-axis); a pure rotation can't be seen from
    gravity's magnitude, so 9 parameters: 6 in M + 3 in b."""
    x, y, z = P.T
    # 1) algebraic fit of the ellipsoid x'Ax + 2v'x = 1 (linear least squares)
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z])
    p = np.linalg.lstsq(D, np.ones(len(P)), rcond=None)[0]
    A = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
    b = -np.linalg.solve(A, p[6:9])                # centre of the ellipsoid = bias
    A = A / (1 + b @ A @ b)                        # now (a-b)'A(a-b) = 1
    w, V = np.linalg.eigh(A)
    if (w <= 0).any():
        raise ValueError("poses don't cover enough directions")
    M = V @ np.diag(np.sqrt(w)) @ V.T              # A = M'M  ->  |M(a-b)| = 1
    # 2) refine on the real error |M(a-b)| - 1 (Gauss-Newton)
    return refine(P, M, b)


def fit_residuals(P, M, b):
    return np.linalg.norm((P - b) @ M.T, axis=1) - 1


def _pack(M, b):
    return np.r_[M[0, 0], M[1, 1], M[2, 2], M[0, 1], M[0, 2], M[1, 2], b]


def _unpack(q):
    M = np.array([[q[0], q[3], q[4]], [q[3], q[1], q[5]], [q[4], q[5], q[2]]])
    return M, q[6:9].copy()


def refine(P, M, b, iters=20):
    q = _pack(M, b)
    for _ in range(iters):
        r = fit_residuals(P, *_unpack(q))
        J = np.empty((len(P), 9))
        for j in range(9):                         # numerical Jacobian
            dq = np.zeros(9)
            dq[j] = 1e-6
            J[:, j] = (fit_residuals(P, *_unpack(q + dq)) - r) / 1e-6
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        q += step
        if np.abs(step).max() < 1e-10:
            break
    return _unpack(q)


def turn_angle(d, axis, gyro_bias):
    """Integrate the bias-corrected rate about one axis -> degrees turned."""
    t = d[:, 0]
    dt = np.diff(t)
    dt[dt < 0] += 2**32 * 1e-6                    # micros() wrap-around
    w = d[:, 4 + axis] - gyro_bias[axis]
    return float(np.sum((w[1:] + w[:-1]) / 2 * dt))  # trapezoid


# ---- serial reading in the background ----

samples = []
lock = threading.Lock()


def reader(ser):
    while True:
        parts = ser.readline().decode(errors="ignore").strip().split(",")
        if len(parts) != 8:
            continue
        try:
            v = [float(x) for x in parts]
        except ValueError:
            continue
        v[0] *= 1e-6                              # us -> s
        with lock:
            samples.append(v)


def mark():
    with lock:
        return len(samples)


def since(start):
    with lock:
        return np.array(samples[start:])


def record(seconds):
    start = mark()
    time.sleep(seconds)
    return since(start)


def collect_poses(rate, gyro_bias, target=24, min_angle=20):
    """Auto-capture: every time the board rests still for 1 s in a direction at least
    min_angle away from all earlier poses, its mean acceleration is saved."""
    poses = []
    n = int(rate)
    try:
        while len(poses) < target:
            time.sleep(0.25)
            with lock:
                d = np.array(samples[-n:])
            g, a = d[:, 4:7], d[:, 1:4]
            if (g.std(0).max() > 1.0 or a.std(0).max() > 0.015
                    or np.abs(g.mean(0) - gyro_bias).max() > 1.0):
                continue                           # moving
            m = a.mean(0)
            u = m / np.linalg.norm(m)
            if any(np.degrees(np.arccos(np.clip(u @ p / np.linalg.norm(p), -1, 1))) < min_angle
                   for p in poses):
                continue                           # too close to a pose we have
            poses.append(m)
            octants = len({tuple(p > 0) for p in poses})
            print(f"   pose {len(poses):2d}/{target}  direction {np.round(u, 2)}  octants {octants}/8")
    except KeyboardInterrupt:
        print("   stopped early")
    return np.array(poses)


def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == 0x2341:                       # Arduino USB vendor id
            return p.device
    raise SystemExit("Nicla not found - give the port, e.g.  python imu_calib.py COM5")


def main():
    import serial

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, 115200, timeout=1)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()

    d = record(2)
    if len(d) < 100:
        raise SystemExit("no IMU data - is imu_calib.ino uploaded and Serial Monitor closed?")
    rate = len(d) / 2
    print(f"connected on {port}, {rate:.0f} samples/s, {d[:, 7].mean():.1f} degC")
    print("tip: let the board run ~5 min first, the gyro bias moves while it warms up\n")

    # 1) gyro bias
    while True:
        input("1) GYRO BIAS: put the board on a table, don't touch it, press Enter (20 s)")
        time.sleep(1)                             # let the Enter-press vibration die out
        d = record(20)
        if is_still(d):
            break
        print("   it moved - try again")
    gyro_bias = d[:, 4:7].mean(0)
    gyro_noise = d[:, 4:7].std(0)
    temp = float(d[:, 7].mean())
    print(f"   gyro bias {np.round(gyro_bias, 3)} dps, noise {np.round(gyro_noise, 3)} dps\n")

    # 2) accelerometer, 6 faces
    faces = {}
    names = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    print("2) ACCEL: rest the board on each of its 6 sides (best: inside a small box).")
    while len(faces) < 6:
        missing = ", ".join(n for n in names if n not in faces)
        input(f"   new side up, still, press Enter ({len(faces)}/6, missing {missing})")
        time.sleep(1)
        d = record(5)
        if not is_still(d):
            print("   it moved - try again")
            continue
        m = d[:, 1:4].mean(0)
        name = face_name(m)
        if np.abs(m).max() < 0.9:
            print(f"   tilted ({np.round(m, 3)} g) - put it flat on a side")
            continue
        if name in faces:
            print(f"   {name} is already done - turn it to a missing side")
            continue
        faces[name] = m
        print(f"   {name} up: {np.round(m, 4)} g")
    acc_bias, acc_scale = accel_from_faces(faces)
    resid = [abs(np.linalg.norm((faces[n] - acc_bias) / acc_scale) - 1) * 1000 for n in names]
    print(f"   accel bias {np.round(acc_bias * 1000, 1)} mg, scale {np.round(acc_scale, 4)}")
    print(f"   check: |a| error per side after calibration, max {max(resid):.1f} mg\n")

    # 3) full ellipsoid fit from many random poses (adds cross-axis, checks step 2)
    acc_M, acc_bias_fit = None, None
    fit_report = {}
    if input("3) ACCEL FIT from many random poses. Do it? [Y/n] ").strip().lower() != "n":
        print("   turn the board to a new angle and hold or lean it still ~2 s, repeat.")
        print("   cover all directions, upside down too. Ctrl+C finishes early.")
        rnd = collect_poses(rate, gyro_bias)
        P = np.vstack([np.array(list(faces.values()))] + ([rnd] if len(rnd) else []))
        if len(P) < 12:
            print("   too few poses for the fit, skipped")
        else:
            try:
                acc_M, acc_bias_fit = ellipsoid_fit(P)
            except ValueError as e:
                print("   fit failed:", e)
        if acc_M is not None:
            test = rnd if len(rnd) else P
            r6 = np.linalg.norm((test - acc_bias) / acc_scale, axis=1) - 1
            rf = fit_residuals(test, acc_M, acc_bias_fit)
            rms = lambda r: np.sqrt(np.mean(r ** 2)) * 1000
            cross = acc_M / np.sqrt(np.outer(np.diag(acc_M), np.diag(acc_M)))
            print(f"   |a| error on the random poses: 6-side {rms(r6):.1f} mg, full fit {rms(rf):.1f} mg")
            fit_report = {"rms_6side_mg": round(rms(r6), 2), "rms_fit_mg": round(rms(rf), 2),
                          "n_random_poses": len(rnd), "poses_g": P.round(6).tolist()}
            print(f"   bias  fit {np.round(acc_bias_fit * 1000, 1)} mg   6-side {np.round(acc_bias * 1000, 1)} mg")
            print(f"   scale fit {np.round(1 / np.diag(acc_M), 4)}   6-side {np.round(acc_scale, 4)}")
            print(f"   cross-axis  XY {cross[0, 1] * 100:.2f}%  XZ {cross[0, 2] * 100:.2f}%  YZ {cross[1, 2] * 100:.2f}%")
        print()

    # 4) gyro scale (optional)
    gyro_scale = np.ones(3)
    if input("4) GYRO SCALE needs exact 360 deg turns. Do it? [y/N] ").strip().lower() == "y":
        for i, a in enumerate("XYZ"):
            while True:
                input(f"   {a}: board still, press Enter, turn it exactly 360 deg about {a}, stop")
                start = mark()
                input("   ...press Enter when it is still again")
                angle = turn_angle(since(start), i, gyro_bias)
                s = 360.0 / abs(angle) if angle else 0
                print(f"   measured {angle:.1f} deg -> scale {s:.4f}")
                if 0.9 < s < 1.1:
                    gyro_scale[i] = s
                    break
                print("   more than 10% off - not a clean 360 deg turn about this axis, retry")
        print()

    result = {
        "settings": "LSM6DSOX 416 Hz, +-4 g, +-2000 dps",
        "temp_C": round(temp, 2),
        "gyro_bias_dps": gyro_bias.round(5).tolist(),
        "gyro_noise_dps": gyro_noise.round(5).tolist(),
        "gyro_scale": gyro_scale.round(5).tolist(),
        "accel_bias_g": acc_bias.round(6).tolist(),
        "accel_scale": acc_scale.round(6).tolist(),
        "apply": "a = (a_raw - accel_bias) / accel_scale;  w = (w_raw - gyro_bias) * gyro_scale",
    }
    if acc_M is not None:
        result["accel_M"] = acc_M.round(6).tolist()
        result["accel_bias_fit_g"] = acc_bias_fit.round(6).tolist()
        result["apply_fit"] = "a = accel_M @ (a_raw - accel_bias_fit)"
        result["fit_check"] = fit_report
    with open("imu_calib.json", "w") as fh:
        json.dump(result, fh, indent=2)
    print("saved imu_calib.json\n\n// paste into the flight code:")

    def arr(name, v, fmt):
        print(f"const float {name}[3] = {{{', '.join(format(x, fmt) for x in v)}}};")

    arr("GYRO_BIAS", gyro_bias, ".5f")
    arr("GYRO_SCALE", gyro_scale, ".5f")
    arr("ACC_BIAS", acc_bias, ".6f")
    arr("ACC_SCALE", acc_scale, ".6f")
    if acc_M is not None:
        print("\n// full fit (use instead of ACC_BIAS/ACC_SCALE): a = ACC_M * (a_raw - ACC_BIAS_FIT)")
        arr("ACC_BIAS_FIT", acc_bias_fit, ".6f")
        rows = ", ".join("{" + ", ".join(f"{x:.6f}" for x in r) + "}" for r in acc_M)
        print(f"const float ACC_M[3][3] = {{{rows}}};")


if __name__ == "__main__":
    main()
