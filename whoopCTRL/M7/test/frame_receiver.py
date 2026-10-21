# Receives camera frames + optical flow from test.ino over USB serial and shows them live.
#   window "Nicla camera + flow": the image, tracked points (green = inlier, red = outlier),
#     their motion arrows and trails, the frame motion (cyan arrow from the lens centre)
#   window "flow path": the frame motion added up over time (drift check)
#   q = quit, s = save the current frame as PNG into ./frames, c = clear trails and path
#   r = ask the board for one raw Bayer image (whole camera image), shown and saved
# pip install pyserial numpy opencv-python
# python frame_receiver.py          (auto-finds the Nicla)
# python frame_receiver.py COM5     (or give the port)

import binascii
import math
import os
import struct
import sys
import time
from collections import deque

import numpy as np

MAGIC = b"\xA5\x5A"
PKT_FRAME, PKT_LOG, PKT_RAW, PKT_CMD, PKT_FLOW = 1, 2, 3, 4, 5   # same as comm::PacketType
FRAME_HDR = struct.Struct("<5I2H4f")    # FrameHeader in test.ino (40 bytes)
RAW_HDR = struct.Struct("<I2H")         # RawHeader (8 bytes)
FLOW_HDR = struct.Struct("<4I7f3HB")    # FlowHeader (51 bytes)
POINT = struct.Struct("<2H4fB")         # PointMsg (21 bytes)
MAX_LEN = 1 << 20                       # anything longer is garbage, not a packet

SCALE = 5                               # window zoom: 100x100 -> 500x500
ARROW_GAIN = 3                          # point / frame motion arrows drawn this many times longer
TRAIL = 12                              # trail length, frames
PATH_SIZE = 400                         # flow path window, pixels


def make_packet(ptype, payload):
    """Build one packet for the board (same format as comm::Communication::send)."""
    body = bytes([ptype]) + struct.pack("<I", len(payload)) + payload
    return MAGIC + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def parse_packets(buf):
    """Take complete, CRC-checked packets out of `buf` (bytearray, consumed in place).
    Yields (type, payload). Garbage and broken packets are skipped (resync on the magic)."""
    while True:
        i = buf.find(MAGIC)
        if i < 0:
            del buf[:max(0, len(buf) - 1)]          # keep a possible half magic byte
            return
        del buf[:i]
        if len(buf) < 7:
            return
        length = struct.unpack_from("<I", buf, 3)[0]
        if length > MAX_LEN:
            del buf[:1]                              # impossible length: false start
            continue
        total = 7 + length + 2
        if len(buf) < total:
            return                                   # wait for the rest
        body = bytes(buf[2:7 + length])             # type + len + payload
        crc = buf[7 + length] | (buf[8 + length] << 8)
        if binascii.crc_hqx(body, 0xFFFF) != crc:    # CRC-16/CCITT-FALSE, same as the board
            del buf[:1]                              # false start, look for the next magic
            continue
        del buf[:total]
        yield body[0], body[5:]


def decode_frame(payload):
    """FRAME payload -> (info dict, grey image)."""
    seq, t_end, grab_us, proc_us, errors, w, h, fx, fy, cx, cy = FRAME_HDR.unpack_from(payload)
    img = np.frombuffer(payload, np.uint8, count=w * h, offset=FRAME_HDR.size).reshape(h, w)
    info = dict(seq=seq, t_end=t_end, grab_us=grab_us, proc_us=proc_us, errors=errors,
                w=w, h=h, fx=fx, fy=fy, cx=cx, cy=cy)
    return info, img


def decode_raw(payload):
    """RAW payload -> (seq, raw Bayer image)."""
    seq, w, h = RAW_HDR.unpack_from(payload)
    return seq, np.frombuffer(payload, np.uint8, count=w * h, offset=RAW_HDR.size).reshape(h, w)


def decode_flow(payload):
    """FLOW payload -> (result dict, list of point dicts)."""
    (seq, t0, t1, flow_us, tx, ty, angle, scale, rms, fx, fy,
     n_tracked, n_inliers, count, valid) = FLOW_HDR.unpack_from(payload)
    res = dict(seq=seq, t0=t0, t1=t1, flow_us=flow_us, tx=tx, ty=ty, angle=angle, scale=scale,
               rms=rms, fx=fx, fy=fy, tracked=n_tracked, inliers=n_inliers, valid=bool(valid))
    body = payload[FLOW_HDR.size:FLOW_HDR.size + count * POINT.size]
    pts = [dict(id=i, age=a, x=x, y=y, dx=dx, dy=dy, inlier=bool(inl))
           for i, a, x, y, dx, dy, inl in POINT.iter_unpack(body)]
    return res, pts


class FlowView:
    """Keeps point trails and the summed frame motion, and draws them."""

    def __init__(self):
        self.trails = {}                             # point id -> recent positions
        self.path = [(0.0, 0.0)]                     # summed frame motion, image pixels
        self.last_seq = None

    def clear(self):
        self.trails.clear()
        self.path = [(0.0, 0.0)]

    def add(self, res, pts):
        if res["seq"] == self.last_seq:
            return
        self.last_seq = res["seq"]
        seen = set()
        for p in pts:
            self.trails.setdefault(p["id"], deque(maxlen=TRAIL)).append((p["x"], p["y"]))
            seen.add(p["id"])
        for pid in list(self.trails):                # forget points that are gone
            if pid not in seen:
                del self.trails[pid]
        if res["valid"]:
            x, y = self.path[-1]
            self.path.append((x + res["tx"], y + res["ty"]))

    def draw_frame(self, img, info, flow, fps):
        import cv2
        S = SCALE
        view = cv2.cvtColor(cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST),
                            cv2.COLOR_GRAY2BGR)
        cx, cy = info["cx"] * S, info["cy"] * S
        cv2.drawMarker(view, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 16, 1)

        lines = [f"seq {info['seq']}  camera {fps:.1f} fps  grab {info['grab_us'] / 1000:.1f} ms"]
        if flow is not None:
            res, pts = flow
            for p in pts:                            # trails, then arrows + dots on top
                tr = self.trails.get(p["id"])
                if tr and len(tr) > 1:
                    poly = np.array([(int(x * S), int(y * S)) for x, y in tr], np.int32)
                    cv2.polylines(view, [poly], False, (0, 200, 255), 1, cv2.LINE_AA)
            for p in pts:
                colour = (0, 220, 0) if p["inlier"] else (0, 0, 255)
                x, y = p["x"] * S, p["y"] * S
                tip = (int(x + p["dx"] * S * ARROW_GAIN), int(y + p["dy"] * S * ARROW_GAIN))
                cv2.arrowedLine(view, (int(x), int(y)), tip, colour, 1, cv2.LINE_AA, tipLength=0.3)
                cv2.circle(view, (int(x), int(y)), 3, colour, -1, cv2.LINE_AA)
            if res["valid"]:                         # frame motion from the lens centre
                tip = (int(cx + res["tx"] * S * ARROW_GAIN), int(cy + res["ty"] * S * ARROW_GAIN))
                cv2.arrowedLine(view, (int(cx), int(cy)), tip, (255, 255, 0), 2, cv2.LINE_AA,
                                tipLength=0.25)
            dt_ms = ((res["t1"] - res["t0"]) & 0xFFFFFFFF) / 1000
            lines += [f"flow {'VALID' if res['valid'] else 'not valid'}  tracked {res['tracked']}  "
                      f"inliers {res['inliers']}  rms {res['rms']:.2f} px",
                      f"shift ({res['tx']:+.2f}, {res['ty']:+.2f}) px  over {dt_ms:.0f} ms",
                      f"rotation {math.degrees(res['angle']):+.2f} deg  scale {res['scale']:.4f}",
                      f"flow time {res['flow_us'] / 1000:.2f} ms"]
        else:
            lines.append("no flow for this frame")
        for i, t in enumerate(lines):
            cv2.putText(view, t, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
                        cv2.LINE_AA)
            cv2.putText(view, t, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                        cv2.LINE_AA)
        return view

    def draw_path(self):
        import cv2
        canvas = np.full((PATH_SIZE, PATH_SIZE, 3), 30, np.uint8)
        pts = np.array(self.path, np.float32)
        span = max(20.0, float(np.abs(pts).max()) * 1.2)     # auto-zoom, at least +-20 px
        k = (PATH_SIZE / 2) / span
        c = PATH_SIZE / 2
        poly = np.array([(int(c + x * k), int(c + y * k)) for x, y in pts], np.int32)
        cv2.drawMarker(canvas, (int(c), int(c)), (120, 120, 120), cv2.MARKER_CROSS, 14, 1)
        if len(poly) > 1:
            cv2.polylines(canvas, [poly], False, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(canvas, tuple(int(v) for v in poly[-1]), 4, (0, 0, 255), -1, cv2.LINE_AA)
        x, y = self.path[-1]
        cv2.putText(canvas, f"sum of image motion: ({x:+.1f}, {y:+.1f}) px", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"view +-{span:.0f} px   c = clear", (8, PATH_SIZE - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)
        return canvas


def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == 0x2341:                          # Arduino USB vendor id
            return p.device
    raise SystemExit("Nicla not found - give the port, e.g.  python frame_receiver.py COM5")


def main():
    import cv2
    import serial

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, 921600, timeout=0.05)
    print(f"listening on {port} - q quits, s saves a frame, c clears, r grabs one raw Bayer image")

    buf = bytearray()
    history = []                                     # (seq, t_end_us) of recent frames
    flows = {}                                       # seq -> (result, points)
    view = FlowView()
    last_rx = time.time()
    warned = False
    printed_k = False
    frame = None

    while True:
        buf += ser.read(ser.in_waiting or 1)
        for ptype, payload in parse_packets(buf):
            last_rx = time.time()
            warned = False
            if ptype == PKT_LOG:
                print("[board]", payload.decode(errors="replace"))
            elif ptype == PKT_FRAME:
                info, img = decode_frame(payload)
                if not printed_k:
                    print(f"crop {info['w']}x{info['h']}  fx {info['fx']:.1f} fy {info['fy']:.1f} "
                          f"cx {info['cx']:.1f} cy {info['cy']:.1f}")
                    printed_k = True
                history = (history + [(info["seq"], info["t_end"])])[-30:]
                frame = (info, img)
            elif ptype == PKT_FLOW:
                res, pts = decode_flow(payload)
                flows[res["seq"]] = (res, pts)
                for old in [s for s in flows if s < res["seq"] - 5]:
                    del flows[old]
                view.add(res, pts)
            elif ptype == PKT_RAW:
                seq, raw = decode_raw(payload)
                os.makedirs("frames", exist_ok=True)
                path = os.path.join("frames", f"raw_{seq:06d}.png")
                cv2.imwrite(path, raw)
                print(f"raw Bayer {raw.shape[1]}x{raw.shape[0]} saved to {path}")
                cv2.imshow("raw Bayer (2x)", cv2.resize(raw, None, fx=2, fy=2,
                                                        interpolation=cv2.INTER_NEAREST))

        if time.time() - last_rx > 3 and not warned:
            print("no packets for 3 s - is test.ino running and Serial Monitor closed?")
            warned = True

        if frame is not None:
            info, img = frame
            fps = 0.0
            if len(history) > 1:
                dseq = history[-1][0] - history[0][0]
                dt = ((history[-1][1] - history[0][1]) & 0xFFFFFFFF) / 1e6   # board clock, wrap-safe
                fps = dseq / dt if dt > 0 else 0.0
            cv2.imshow("Nicla camera + flow", view.draw_frame(img, info, flows.get(info["seq"]), fps))
            cv2.imshow("flow path", view.draw_path())

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("c"):
            view.clear()
        if key == ord("r"):
            ser.write(make_packet(PKT_CMD, b"r"))
        if key == ord("s") and frame is not None:
            os.makedirs("frames", exist_ok=True)
            path = os.path.join("frames", f"frame_{frame[0]['seq']:06d}.png")
            cv2.imwrite(path, frame[1])
            print("saved", path)

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
