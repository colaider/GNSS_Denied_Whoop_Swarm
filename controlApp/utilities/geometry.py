# geometry - 3D helpers, numpy, for many points at once. Frames as on the drone: x forward, y left, z up.

import numpy as np


def rotate(roll, pitch, yaw, v):        # body -> start frame, ZYX like IMU::Attitude, radians; v: (N, 3)
    v = np.asarray(v, float).reshape(-1, 3)
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    return np.column_stack([x * cy - y * sy, x * sy + y * cy, z])


def project(p, az, el):                 # start-frame points (N, 3) -> screen (right, up), seen from azimuth az,
    p = np.asarray(p, float).reshape(-1, 3)   # elevation el (radians); the same view as hoverTest.py's 3D views
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    right = -x * np.sin(az) + y * np.cos(az)
    up = -np.sin(el) * (x * np.cos(az) + y * np.sin(az)) + z * np.cos(el)
    return right, up


def ring(centre, r, n=16):              # a circle in the x-y plane: (n + 1, 3), closed
    a = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([centre[0] + r * np.cos(a), centre[1] + r * np.sin(a), np.full(n + 1, centre[2])])
