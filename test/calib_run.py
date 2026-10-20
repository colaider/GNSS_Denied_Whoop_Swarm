# Calibrate the Nicla Vision camera from the saved chessboard images.
# Run calib_capture.py first, then this. Produces calibration.npz.
# python calib_run.py            -> calibrate + show undistorted preview
# python calib_run.py --no-show  -> calibrate only
# pip install numpy opencv-python

import glob
import sys

import numpy as np
import cv2

IN = "calib_images"
CHESS = (8, 6)               # INNER corners (columns, rows) - must match capture
SQUARE_MM = 25               # square size in mm (only scales poses, not intrinsics)
CAL_W = 640                  # width of the camera mode the images were taken in

# GC2145 modes in the Arduino driver: width -> (subsample s, window x0, window y0)
# each mode is a centred crop of the 1600x1200 sensor, shrunk by s
MODES = {160: (4, 480, 360), 320: (3, 320, 240), 640: (2, 160, 120)}

# 3D coordinates of the chessboard corners in the board's own frame
objp = np.zeros((CHESS[0] * CHESS[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS[0], 0:CHESS[1]].T.reshape(-1, 2) * SQUARE_MM

objpoints, imgpoints, used = [], [], []
size = None
for p in sorted(glob.glob(IN + "/img_*.png")):      # only real captures
    gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
    size = gray.shape[::-1]   # (width, height)
    found, corners = cv2.findChessboardCornersSB(gray, CHESS, flags=cv2.CALIB_CB_ACCURACY)
    if not found:
        print("no board:", p)
        continue
    objpoints.append(objp)
    imgpoints.append(corners)
    used.append(p)

if len(objpoints) < 5:
    raise SystemExit(f"need >=5 good images, got {len(objpoints)}")


def calibrate(flags):
    rms, K, D, _, _, std, _, per_view = cv2.calibrateCameraExtended(
        objpoints, imgpoints, size, None, None, flags=flags)
    return rms, K, D, std.ravel(), per_view.ravel()


# full model (k1 k2 p1 p2 k3) vs simpler model with k3 fixed to 0
rms5, K5, D5, std5, pv5 = calibrate(0)
rms4, K4, D4, std4, pv4 = calibrate(cv2.CALIB_FIX_K3)

print("\n--- model comparison ---")
print(f"full model  (k1 k2 p1 p2 k3): RMS {rms5:.4f} px")
print(f"k3 fixed    (k1 k2 p1 p2)   : RMS {rms4:.4f} px")
if rms4 <= rms5 * 1.05:      # simpler model is (almost) as good -> safer at the edges
    model, rms, K, D, std, pv = "k3 fixed", rms4, K4, D4, std4, pv4
else:
    model, rms, K, D, std, pv = "full", rms5, K5, D5, std5, pv5
print("using:", model)

print("\n--- error per image (RMS px, worst first) ---")
med = np.median(pv)
for i in np.argsort(pv)[::-1]:
    flag = "  <- check / retake" if pv[i] > 2 * med else ""
    print(f"{pv[i]:.3f}  {used[i]}{flag}")


def convert(K, new_w):
    """Convert intrinsics from the calibrated mode to another camera mode."""
    s1, x1, y1 = MODES[CAL_W]
    s2, x2, y2 = MODES[new_w]
    K2 = K.copy()
    K2[0, 0] = K[0, 0] * s1 / s2                     # fx
    K2[1, 1] = K[1, 1] * s1 / s2                     # fy
    K2[0, 2] = (x1 + s1 * K[0, 2] - x2) / s2         # cx
    K2[1, 2] = (y1 + s1 * K[1, 2] - y2) / s2         # cy
    return K2


print("\n--- calibration result ---")
print("images used :", len(objpoints))
print("image size  :", size)
print(f"RMS error   : {rms:.4f} px   (< 0.5 good, < 0.3 very good)")
print(f"fx {K[0,0]:.2f} +/- {std[0]:.2f}   fy {K[1,1]:.2f} +/- {std[1]:.2f}")
print(f"cx {K[0,2]:.2f} +/- {std[2]:.2f}   cy {K[1,2]:.2f} +/- {std[3]:.2f}")
print("distortion (k1 k2 p1 p2 k3):", np.round(D.ravel(), 5))

out = {"camera_matrix": K, "dist_coeff": D, "image_size": size, "cal_width": CAL_W}
print("\n--- intrinsics per camera mode (distortion is the same for all) ---")
for w in sorted(MODES):
    Kw = convert(K, w)
    out[f"K_{w}"] = Kw
    print(f"{w}x{w * 3 // 4}: fx {Kw[0,0]:.2f} fy {Kw[1,1]:.2f} cx {Kw[0,2]:.2f} cy {Kw[1,2]:.2f}")

np.savez("calibration.npz", **out)
print("\nsaved calibration.npz (camera_matrix, dist_coeff, K_160, K_320, K_640)")

if "--no-show" not in sys.argv:
    img = cv2.imread(used[0])
    cv2.imshow("original", img)
    cv2.imshow("undistorted", cv2.undistort(img, K, D))
    print("press any key in the image window to close")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
