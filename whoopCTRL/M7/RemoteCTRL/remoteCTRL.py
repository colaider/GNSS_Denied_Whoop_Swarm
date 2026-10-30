# remoteCTRL - fly the whoop from the PC keyboard over WiFi.
#   PC --UDP--> M7 (M7/RemoteCTRL) --core link--> M4 (M4/RemoteCTRL) --MSP--> Betaflight
#
#   python remoteCTRL.py              the Nicla's own network (WIFI_AP 1): join "WhoopCTRL" first
#   python remoteCTRL.py 192.168.1.57 the Nicla's address on your network (WIFI_AP 0, see its USB log)
#
# Keys (click the window first; it must have focus):
#   Enter      ARM (throttle goes to 0 first)        Space / Esc   DISARM NOW (Esc also quits)
#   W / S      throttle up / down (stays where left)  A / D         yaw left / right (springs back)
#   arrows     pitch forward / back, roll left / right (spring back)
#   1 2 3      ACRO / ANGLE / HORIZON                  [ / ]         tilt smaller / bigger
#   bench test, DISARMED, PROPS OFF:  Z X C V = motor 1 2 3 4 while held,  B = all four
# Closing the window disarms. Commands go out at 50 Hz; if they stop, the drone levels, lowers
# the throttle and disarms by itself (Betaflight class, commandTimeoutMs).

import binascii
import socket
import struct
import sys
import time
import tkinter as tk

PORT = 4210
RATE_HZ = 50
PKT_LOG, PKT_REMOTE, PKT_REMOTE_STATE = 2, 14, 15
RC_STICKS, RC_ARM, RC_DISARM, RC_MOTOR = 0, 1, 2, 3
MODES = ["ACRO", "ANGLE", "HORIZON"]

THR_RATE = 0.4       # throttle change per second while W / S is held
THR_MAX = 0.8        # throttle never above this
YAW = 0.4            # yaw stick while A / D is held
TILT_STEPS = [0.1, 0.15, 0.2, 0.3, 0.4]   # roll / pitch stick (ANGLE: x 55 deg; 0.2 = 11 deg)
BENCH_SPEED = 0.10   # motor test speed (1100 us)

REMOTE_FMT = "<H3B4f"            # msg::RemoteCommand
STATE_FMT = "<IH4BI4f8H"         # msg::RemoteState
BLOCKERS = ["NO_GYRO", "FAILSAFE", "RX_LOSS", "BAD_RX_RECOVERY", "BOX_FAILSAFE", "RUNAWAY_TAKEOFF",
            "CRASH", "THROTTLE", "ANGLE", "BOOT_GRACE", "NO_PREARM", "LOAD", "CALIBRATING", "CLI",
            "CMS_MENU", "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SWITCH", "RPM_FILTER", "REBOOT_REQUIRED",
            "DSHOT_BITBANG", "ACC_CALIBRATION", "MOTOR_PROTOCOL", "CRASHFLIP", "ALTHOLD", "POSHOLD"]


def packet(ptype, payload):
    """comm::Communication packet: A5 5A | type | len u32 | payload | crc16 (CCITT-FALSE)."""
    head = struct.pack("<BI", ptype, len(payload))
    return b"\xA5\x5A" + head + payload + struct.pack("<H", binascii.crc_hqx(head + payload, 0xFFFF))


def parse(buf):
    """Complete packets from buf (bytearray, consumed): list of (type, payload)."""
    out = []
    while True:
        i = buf.find(b"\xA5\x5A")
        if i < 0:
            del buf[:-1]
            return out
        if len(buf) < i + 7:
            del buf[:i]
            return out
        ptype, n = struct.unpack_from("<BI", buf, i + 2)
        if n > 2048:
            del buf[:i + 2]
            continue
        if len(buf) < i + 9 + n:
            del buf[:i]
            return out
        body = bytes(buf[i + 2:i + 7 + n])
        crc = struct.unpack_from("<H", buf, i + 7 + n)[0]
        if binascii.crc_hqx(body, 0xFFFF) == crc:
            out.append((ptype, body[5:]))
            del buf[:i + 9 + n]
        else:
            del buf[:i + 2]


def blocker_names(flags, count):
    return [("ARM_SWITCH" if b == count - 1 else (BLOCKERS[b] if b < len(BLOCKERS) else f"bit{b}"))
            for b in range(32) if flags >> b & 1]


class Remote:
    def __init__(self, ip):
        self.addr = (ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.rx = bytearray()
        self.seq = 0
        self.held = set()
        self.throttle = 0.0
        self.mode = 1                         # ANGLE
        self.tilt = 2                         # index in TILT_STEPS
        self.arm_until = self.disarm_until = 0.0
        self.state = None
        self.state_time = 0.0
        self.logs = []
        self.last = time.time()
        self.sent = (0, 0, 0, 0, 0)

        self.root = tk.Tk()
        self.root.title(f"remoteCTRL -> {ip}:{PORT}")
        self.text = tk.Label(self.root, font=("Consolas", 11), justify="left", anchor="nw", width=78, height=24)
        self.text.pack(padx=8, pady=8)
        self.root.bind("<KeyPress>", self.press)
        self.root.bind("<KeyRelease>", lambda e: self.held.discard(e.keysym.lower()))
        self.root.bind("<FocusOut>", lambda e: self.held.clear())      # no stuck keys
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(0, self.tick)

    # ---------------------------------------------------------------- keys
    def press(self, e):
        k = e.keysym.lower()
        self.held.add(k)
        now = time.time()
        if k in ("space", "escape"):
            self.throttle = 0.0
            self.disarm_until = now + 0.3     # repeated for 0.3 s: UDP may lose one
            self.arm_until = 0.0
            if k == "escape":
                self.quit()
        elif k == "return":
            self.throttle = 0.0
            self.arm_until = now + 0.5
        elif k in ("1", "2", "3"):
            self.mode = int(k) - 1
        elif k == "bracketleft":
            self.tilt = max(0, self.tilt - 1)
        elif k == "bracketright":
            self.tilt = min(len(TILT_STEPS) - 1, self.tilt + 1)

    def down(self, k):
        return 1.0 if k in self.held else 0.0

    # ---------------------------------------------------------------- 50 Hz
    def tick(self):
        now = time.time()
        dt, self.last = min(0.1, now - self.last), now    # a frozen window must not jump the throttle
        self.throttle = min(THR_MAX, max(0.0, self.throttle + THR_RATE * dt * (self.down("w") - self.down("s"))))
        tilt = TILT_STEPS[self.tilt]
        roll = tilt * (self.down("right") - self.down("left"))
        pitch = tilt * (self.down("up") - self.down("down"))
        yaw = YAW * (self.down("d") - self.down("a"))

        motor_keys = {"z": 0, "x": 1, "c": 2, "v": 3, "b": 255}
        motor = next((m for k, m in motor_keys.items() if k in self.held), None)
        if now < self.disarm_until:
            action, motor, thr = RC_DISARM, 0, 0.0
        elif now < self.arm_until:
            action, motor, thr = RC_ARM, 0, 0.0
        elif motor is not None:
            action, thr = RC_MOTOR, BENCH_SPEED
            roll = pitch = yaw = 0.0
        else:
            action, motor, thr = RC_STICKS, 0, self.throttle
        self.seq = (self.seq + 1) & 0xFFFF
        self.send(packet(PKT_REMOTE, struct.pack(REMOTE_FMT, self.seq, action, self.mode, motor, roll, pitch, yaw, thr)))
        self.sent = (action, roll, pitch, yaw, thr)

        self.receive()
        self.draw(now)
        self.root.after(max(1, int(1000 / RATE_HZ - (time.time() - now) * 1000)), self.tick)

    def send(self, data):
        try:
            self.sock.sendto(data, self.addr)
        except OSError:
            pass                              # not on the Nicla's network yet

    def receive(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, ConnectionResetError, OSError):
                break
            self.rx += data
        for ptype, p in parse(self.rx):
            if ptype == PKT_REMOTE_STATE and len(p) == struct.calcsize(STATE_FMT):
                self.state = struct.unpack(STATE_FMT, p)
                self.state_time = time.time()
            elif ptype == PKT_LOG:
                line = time.strftime("%H:%M:%S ") + p.decode(errors="replace")
                print(line)
                self.logs = (self.logs + [line])[-6:]

    # ---------------------------------------------------------------- screen
    def draw(self, now):
        act = ["fly", "ARM", "DISARM", "MOTOR TEST"][self.sent[0]]
        lines = [f"sending  {act:<10} mode {MODES[self.mode]:<8} tilt {TILT_STEPS[self.tilt]:.2f}",
                 f"sticks   roll {self.sent[1]:+.2f}  pitch {self.sent[2]:+.2f}  yaw {self.sent[3]:+.2f}  "
                 f"throttle {self.sent[4]:.2f}  [{'#' * int(self.sent[4] * 30):<30}]", ""]
        s = self.state
        if s is None or now - self.state_time > 1.0:
            lines.append("NO REPLY from the drone: on its WiFi? M7 + M4 RemoteCTRL uploaded? right address?")
        else:
            t, seq, fc, armed, mode, cnt, blk, vbat, r, p, y = s[:11]
            rc = s[11:]
            lines.append(f"drone    {'FC ok' if fc else 'FC SILENT'}   {['disarmed', 'arming...', 'ARMED'][armed]}"
                         f"   mode {MODES[mode] if mode < 3 else mode}   battery {vbat:.2f} V   packets lost {(self.seq - seq) & 0xFFFF}")
            lines.append(f"attitude roll {r:+6.1f}  pitch {p:+6.1f}  yaw {y:6.1f} deg")
            lines.append("rc sent  " + " ".join(str(v) for v in rc))
            if not armed:
                lines.append("blockers " + (" ".join(blocker_names(blk, cnt)) or "none - ready to arm"))
        lines += ["", "Enter arm | Space disarm | W/S throttle | A/D yaw | arrows tilt | 1 2 3 mode | [ ] tilt",
                  "bench (disarmed, props off): Z X C V motor 1-4, B all | Esc disarm + quit", ""] + self.logs
        self.text.config(text="\n".join(lines))

    def quit(self):
        for i in range(10):                   # disarm, a few times
            self.seq = (self.seq + 1) & 0xFFFF
            self.send(packet(PKT_REMOTE, struct.pack(REMOTE_FMT, self.seq, RC_DISARM, self.mode, 0, 0, 0, 0, 0)))
            time.sleep(0.02)
        self.root.destroy()


if __name__ == "__main__":
    Remote(sys.argv[1] if len(sys.argv) > 1 else "192.168.3.1").root.mainloop()
