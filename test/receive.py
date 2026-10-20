# Connect to Nicla Vision over WiFi: live IMU plot + live camera stream.
# As simple as possible, no safety checks. Auto-reconnects.
# pip install numpy opencv-python matplotlib

import socket
import time
from collections import deque

import numpy as np
import cv2
import matplotlib.pyplot as plt

NICLA_IP = "172.20.10.2"   # IP printed by the Nicla on serial
PORT = 8080
N = 200                    # samples shown in the IMU plot

# --- IMU live plot (built once) ---
labels = ["ax", "ay", "az", "gx", "gy", "gz"]
bufs = [deque([0.0] * N, maxlen=N) for _ in labels]
plt.ion()
fig, (ax_a, ax_g) = plt.subplots(2, 1)
lines = []
for i, lbl in enumerate(labels):
    target = ax_a if i < 3 else ax_g
    (ln,) = target.plot(range(N), list(bufs[i]), label=lbl)
    lines.append(ln)
ax_a.set_title("accel"); ax_a.legend(loc="upper right")
ax_g.set_title("gyro");  ax_g.legend(loc="upper right")
plt.show(block=False)


def stream():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((NICLA_IP, PORT))
    sock.sendall(b"go\n")          # nudge the board's server.available()
    f = sock.makefile("rb")
    print("connected")
    n = 0
    try:
        while True:
            line = f.readline()
            if not line:
                print("stream closed by board")
                return
            header = line.decode(errors="ignore").split()
            # IMU ax ay az gx gy gz TOF dist FRAME WxH len
            if len(header) < 12 or header[0] != "IMU":
                continue
            imu = list(map(float, header[1:7]))
            tof = int(header[8])
            w, h = map(int, header[10].split("x"))
            length = int(header[11])
            if length != w * h * 2:          # RGB565 = 2 bytes/pixel
                continue
            frame = f.read(length)           # exactly one raw frame
            if len(frame) != length:
                continue

            # --- camera stream (decode RGB565 -> BGR) ---
            raw = np.frombuffer(frame, dtype=np.uint8).reshape(h, w, 2)[:, :, ::-1].copy()
            img = cv2.cvtColor(raw, cv2.COLOR_BGR5652BGR)   
            cv2.putText(img, f"ToF {tof}mm", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.imshow("Nicla camera", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

            # --- IMU plot (throttled: redraw every 4th frame) ---
            for i, v in enumerate(imu):
                bufs[i].append(v)
            n += 1
            if n % 4 == 0:
                for i in range(len(lines)):
                    lines[i].set_ydata(bufs[i])
                ax_a.relim(); ax_a.autoscale_view()
                ax_g.relim(); ax_g.autoscale_view()
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
    finally:
        f.close()
        sock.close()               # always drop the socket so board frees it


while True:
    try:
        stream()
    except KeyboardInterrupt:
        break
    except (OSError, ValueError) as e:
        print("connection error:", e)
    print("reconnecting in 1s...")
    time.sleep(1)

cv2.destroyAllWindows()
