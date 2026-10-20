# Capture N full-resolution chessboard images from camera_calib.ino.
# Live view = small preview. SPACE asks the board for ONE full 640x480 raw
# Bayer frame, converts it to grey and saves it (only if the board is found).
# SPACE = capture, q = quit.
# pip install numpy opencv-python

import os
import socket

import numpy as np
import cv2

NICLA_IP = "172.20.10.2"
PORT = 8080
N = 20                       # how many full images to collect
OUT = "calib_images"
CHESS = (8, 6)               # INNER corners (columns, rows)

os.makedirs(os.path.join(OUT, "raw"), exist_ok=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)            # don't hang forever if the board sends nothing
sock.connect((NICLA_IP, PORT))
sock.sendall(b"go\n")
f = sock.makefile("rb")
print("connected, waiting for frames...")

warned = False


def read_frame():
    """Return (kind, image) for one frame. kind 'P' = preview, 'F' = full raw."""
    global warned
    while True:
        try:
            line = f.readline()
        except TimeoutError:
            raise SystemExit("no data from the board for 5 s - open Serial Monitor: "
                             "'grab failed' / 'camera init failed' = camera problem, "
                             "nothing at all = board crashed or wrong sketch")
        if not line:
            raise SystemExit("board closed the connection")
        h = line.decode(errors="ignore").split()
        # FRAME <kind> <w> <h> <len>
        if len(h) < 5 or h[0] != "FRAME":
            if not warned and h and h[0] == "IMU":
                print("board is sending the IMU stream - upload camera_calib.ino instead")
                warned = True
            continue
        kind, w, ht, n = h[1], int(h[2]), int(h[3]), int(h[4])
        data = f.read(n)
        if len(data) != n or n != w * ht:
            continue
        return kind, np.frombuffer(data, np.uint8).reshape(ht, w)


def bayer_to_grey(raw):
    """Flatten the Bayer mosaic without knowing its pattern: scale each of the
    4 colour phases to the same mean. Exact for a neutral (black/white) board."""
    g = raw.astype(np.float32)
    m = g.mean()
    for dy in (0, 1):
        for dx in (0, 1):
            p = g[dy::2, dx::2]
            p *= m / max(p.mean(), 1.0)
    return np.clip(g, 0, 255).astype(np.uint8)


def grab_full():
    """Ask the board for one full frame and return it (raw Bayer)."""
    sock.sendall(b"F")
    for _ in range(30):               # skip previews already in flight
        kind, img = read_frame()
        if kind == "F":
            return img
    return None


count = 0
while count < N:
    kind, img = read_frame()
    if kind != "P":
        continue
    prev = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    # sector-based detector: copes much better with a tiny, soft preview image
    found, corners = cv2.findChessboardCornersSB(prev, CHESS)

    view = cv2.cvtColor(prev, cv2.COLOR_GRAY2BGR)
    if found:
        cv2.drawChessboardCorners(view, CHESS, corners, found)
    view = cv2.resize(view, (img.shape[1] * 4, img.shape[0] * 4), interpolation=cv2.INTER_NEAREST)
    cv2.putText(view, f"{count}/{N}  {'BOARD' if found else 'no board'}  SPACE=capture q=quit",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    cv2.imshow("calib capture (preview)", view)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k == ord(" "):
        raw = grab_full()
        if raw is None:
            print("no full frame received, try again")
            continue
        grey = bayer_to_grey(raw)
        ok, _ = cv2.findChessboardCornersSB(grey, CHESS)
        if not ok:
            dbg = os.path.join(OUT, "last_fail.png")
            cv2.imwrite(dbg, grey)          # keep it so we can see what the camera saw
            print(f"full frame has no chessboard, skipped (saved {dbg})")
            continue
        cv2.imwrite(os.path.join(OUT, "raw", f"raw_{count:02d}.png"), raw)
        p = os.path.join(OUT, f"img_{count:02d}.png")
        cv2.imwrite(p, grey)
        print(f"saved {p}  ({grey.shape[1]}x{grey.shape[0]})")
        count += 1

sock.close()
cv2.destroyAllWindows()
print(f"done: {count} full images in {OUT}/")
