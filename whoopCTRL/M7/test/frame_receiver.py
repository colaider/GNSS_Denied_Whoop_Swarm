# Receives camera frames from test.ino over USB serial and shows them live.
#   q = quit, s = save the current frame as PNG into ./frames
#   r = ask the board for one raw Bayer image (whole camera image), shown and saved
# pip install pyserial numpy opencv-python
# python frame_receiver.py          (auto-finds the Nicla)
# python frame_receiver.py COM5     (or give the port)

import binascii
import os
import struct
import sys
import time

import numpy as np

MAGIC = b"\xA5\x5A"
PKT_FRAME, PKT_LOG, PKT_RAW, PKT_CMD = 1, 2, 3, 4    # same as comm::PacketType
FRAME_HDR = struct.Struct("<5I2H4f")    # must match FrameHeader in test.ino (40 bytes)
RAW_HDR = struct.Struct("<I2H")         # must match RawHeader in test.ino (8 bytes)
MAX_LEN = 1 << 20                       # anything longer is garbage, not a packet
SCALE = 4                               # window zoom: 100x100 -> 400x400


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
    print(f"listening on {port} - q quits, s saves a frame, r grabs one raw Bayer image")

    buf = bytearray()
    history = []                                     # (seq, t_end_us) of recent frames
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
            view = cv2.cvtColor(cv2.resize(img, None, fx=SCALE, fy=SCALE,
                                           interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
            # principal point of the crop (where the lens axis hits the image)
            px, py = int(info["cx"] * SCALE), int(info["cy"] * SCALE)
            cv2.drawMarker(view, (px, py), (0, 0, 255), cv2.MARKER_CROSS, 16, 1)

            fps = 0.0
            if len(history) > 1:
                dseq = history[-1][0] - history[0][0]
                dt = ((history[-1][1] - history[0][1]) & 0xFFFFFFFF) / 1e6   # board clock, wrap-safe
                fps = dseq / dt if dt > 0 else 0.0
            lines = [f"seq {info['seq']}  camera {fps:.1f} fps",
                     f"grab {info['grab_us'] / 1000:.1f} ms  proc {info['proc_us'] / 1000:.2f} ms",
                     f"grab errors {info['errors']}"]
            for i, t in enumerate(lines):
                cv2.putText(view, t, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("Nicla camera test", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
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
