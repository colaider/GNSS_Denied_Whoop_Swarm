# Prints the board's log lines (PKT_LOG) - for M4/test/armTest, which only logs.
# Runs with M7/m7Runner on the M7 (it passes the M4's lines to USB). Close Serial Monitor first.
# python fc_log.py          (auto-finds the Nicla)
# python fc_log.py COM5     (or give the port)

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "M7", "test"))
from frame_receiver import PKT_LOG, find_port, parse_packets  # noqa: E402


def main():
    import serial
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, 921600, timeout=0.05)
    print(f"listening on {port} - Ctrl+C quits")
    buf = bytearray()
    last = time.time()
    try:
        while True:
            buf += ser.read(ser.in_waiting or 1)
            for ptype, payload in parse_packets(buf):
                last = time.time()
                if ptype == PKT_LOG:
                    print(time.strftime("%H:%M:%S"), payload.decode(errors="replace"), flush=True)
            if time.time() - last > 3:
                print("no packets for 3 s - is m7Runner running on the M7?")
                last = time.time()
    except KeyboardInterrupt:
        pass
    ser.close()


if __name__ == "__main__":
    main()
