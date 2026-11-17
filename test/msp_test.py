# Talk to the Betaflight flight controller from the PC with MSP v1 - no Nicla in between.
# Connect the PC either to the FC's own USB port (it always speaks MSP), or with a USB-to-serial
# adapter to the UART pads (adapter TX -> FC RX, adapter RX -> FC TX, GND - GND; Ports tab: MSP on
# that UART, 115200). Close Betaflight Configurator first (it holds the port, and blocks arming).
#
#   python msp_test.py                     lists the serial ports
#   python msp_test.py COM7 info           who it is, battery, attitude, modes, what blocks arming
#   python msp_test.py COM7 rc             sends RC (sticks centred, throttle low, AUX1 low) at 50 Hz
#                                          for 5 s and prints what the FC received (MSP_RC)
#   python msp_test.py COM7 arm            arm switch low 1 s, high: armed 5 s at zero throttle
#                                          (MOTORS SPIN AT IDLE - PROPS OFF), then disarm
#   python msp_test.py COM7 motor 1 1100   motor test, disarmed: motor 1 at 1100 for 2 s (PROPS OFF)
# Ctrl+C at any time: arm switch low + all motors 1000 are sent before it quits.
#
# rc / arm need RX_MSP in Betaflight (CLI: feature -RX_SERIAL, feature RX_MSP, save) and ARM on
# AUX1 (CLI: aux 0 0 0 1700 2100 0 0, save). info and motor work without.
# pip install pyserial

import struct
import sys
import time

MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION = 1, 2, 3
MSP_STATUS, MSP_RC, MSP_ATTITUDE, MSP_ANALOG, MSP_BOXIDS = 101, 105, 108, 110, 119
MSP_SET_RAW_RC, MSP_SET_MOTOR = 200, 214

BOX_NAMES = {0: "ARM", 1: "ANGLE", 2: "HORIZON", 3: "ALTHOLD", 11: "POSHOLD", 13: "BEEPER",
             35: "CRASHFLIP", 50: "MSP OVERRIDE"}
BLOCKERS = ["NO_GYRO", "FAILSAFE", "RX_LOSS", "BAD_RX/NOT_DISARMED", "BOX_FAILSAFE", "RUNAWAY_TAKEOFF",
            "CRASH", "THROTTLE", "ANGLE", "BOOT_GRACE", "NO_PREARM", "LOAD", "CALIBRATING", "CLI", "CMS_MENU",
            "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SWITCH", "RPM_FILTER/DSHOT_TELEM", "REBOOT_REQUIRED",
            "DSHOT_BITBANG", "ACC_CALIBRATION", "MOTOR_PROTOCOL", "CRASHFLIP", "ALTHOLD", "POSHOLD", "AUTOPILOT"]


def frame(cmd, payload=b""):
    """MSP v1 request: '$M<' size cmd payload checksum (XOR of size, cmd and payload)."""
    s = len(payload) ^ cmd
    for b in payload:
        s ^= b
    return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([s])


class Msp:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.buf = bytearray()
        self.replies = {}                 # cmd -> newest payload

    def send(self, cmd, payload=b""):
        self.ser.write(frame(cmd, payload))

    def poll(self):
        """Reads what arrived, keeps every complete reply ('$M>') by command."""
        self.buf += self.ser.read(self.ser.in_waiting or 0)
        while True:
            i = self.buf.find(b"$M")
            if i < 0 or len(self.buf) < i + 5:
                return
            d, size, cmd = self.buf[i + 2], self.buf[i + 3], self.buf[i + 4]
            if len(self.buf) < i + 6 + size:
                return
            data = bytes(self.buf[i + 5:i + 5 + size])
            s = size ^ cmd
            for b in data:
                s ^= b
            ok = d == ord(">") and s == self.buf[i + 5 + size]
            if ok:
                self.replies[cmd] = data
            elif d == ord("!"):
                print(f"  FC: error reply to command {cmd}")
            del self.buf[:i + 6 + size if d in (ord(">"), ord("!")) else i + 2]

    def ask(self, cmd, timeout=0.5):
        """Sends a request and waits for its reply (None if none came)."""
        self.replies.pop(cmd, None)
        self.send(cmd)
        t = time.time()
        while time.time() - t < timeout:
            self.poll()
            if cmd in self.replies:
                return self.replies[cmd]
            time.sleep(0.005)
        return None

    def rc(self, roll=1500, pitch=1500, throttle=1000, yaw=1500, aux1=1000, aux2=1000):
        self.send(MSP_SET_RAW_RC, struct.pack("<8H", roll, pitch, throttle, yaw, aux1, aux2, 1000, 1000))

    def motors(self, values):
        self.send(MSP_SET_MOTOR, struct.pack("<8H", *(list(values) + [1000] * 8)[:8]))

    def safe(self):
        """Arm switch low, motors stopped - sent a few times to be sure."""
        for _ in range(5):
            self.rc()
            self.motors([1000] * 8)
            time.sleep(0.02)


def status(d):
    """MSP_STATUS -> (armed, list of arming blockers)."""
    if not d or len(d) < 16:
        return None, []
    extra = d[15]
    a = 16 + extra
    armed = bool(d[6] & 1)                # ARM is always the first mode bit
    if len(d) < a + 5:
        return armed, []
    count, flags = d[a], struct.unpack_from("<I", d, a + 1)[0]
    names = []
    for i in range(32):
        if flags & (1 << i):
            names.append("ARM_SWITCH" if i == count - 1 else (BLOCKERS[i] if i < len(BLOCKERS) else f"bit{i}"))
    return armed, names


def info(m):
    v = m.ask(MSP_FC_VARIANT)
    ver = m.ask(MSP_FC_VERSION)
    api = m.ask(MSP_API_VERSION)
    if v is None:
        print("no answer from the FC: wrong port? TX/RX crossed? MSP enabled on that UART? Configurator closed?")
        return False
    line = f"FC: {v.decode(errors='replace')}"
    if ver and len(ver) >= 3:
        line += f" {ver[0]}.{ver[1]}.{ver[2]}"
    if api and len(api) >= 3:
        line += f"  MSP API {api[1]}.{api[2]}"
    print(line)
    an = m.ask(MSP_ANALOG)
    if an and len(an) >= 9:
        print(f"battery {struct.unpack_from('<H', an, 7)[0] / 100:.2f} V")
    at = m.ask(MSP_ATTITUDE)
    if at:
        r, p, y = struct.unpack_from("<3h", at)
        print(f"attitude: roll {r / 10:.1f}  pitch {p / 10:.1f}  yaw {y} deg")
    boxes = m.ask(MSP_BOXIDS)
    if boxes:
        print("modes it has: " + ", ".join(BOX_NAMES.get(b, str(b)) for b in boxes))
    armed, why = status(m.ask(MSP_STATUS))
    print(f"{'ARMED' if armed else 'disarmed'}, arming blocked by: {' '.join(why) if why else 'nothing'}")
    return True


def run_rc(m):
    print("sending RC 5 s: roll/pitch/yaw 1500, throttle 1000, AUX1 1000 (FC should show these back)")
    t0 = time.time()
    last = 0
    while time.time() - t0 < 5:
        m.rc()
        m.poll()
        if time.time() - last > 0.5:
            last = time.time()
            got = m.ask(MSP_RC, 0.1)
            if got:
                ch = struct.unpack(f"<{len(got) // 2}H", got)
                print("  FC received:", " ".join(str(c) for c in ch[:8]))
            else:
                print("  no MSP_RC reply")
        time.sleep(0.02)
    armed, why = status(m.ask(MSP_STATUS))
    print(f"blocked by: {' '.join(why) if why else 'nothing'}  (RX_LOSS gone = the FC takes our RC)")


def run_arm(m):
    print("PROPS OFF. Arm switch low 1 s, then high for 5 s at zero throttle, then disarm.")
    t0 = time.time()
    last = 0
    while time.time() - t0 < 7:
        t = time.time() - t0
        m.rc(aux1=2000 if 1.0 <= t < 6.0 else 1000)
        m.poll()
        if time.time() - last > 0.5:
            last = time.time()
            armed, why = status(m.ask(MSP_STATUS, 0.1))
            print(f"  t {t:4.1f} s  switch {'HIGH' if 1.0 <= t < 6.0 else 'low '}  "
                  f"{'ARMED' if armed else 'disarmed'}  blocked by: {' '.join(why) if why else 'nothing'}")
        time.sleep(0.02)


def run_motor(m, n, speed):
    print(f"PROPS OFF. Motor {n} at {speed} for 2 s (only works disarmed).")
    vals = [1000] * 8
    vals[n - 1] = speed
    t0 = time.time()
    while time.time() - t0 < 2:
        m.motors(vals)
        m.poll()
        time.sleep(0.05)


def main():
    if len(sys.argv) < 3:
        from serial.tools import list_ports
        print("usage: python msp_test.py PORT info | rc | arm | motor N SPEED   (see the top of this file)")
        print("serial ports:", ", ".join(f"{p.device} ({p.description})" for p in list_ports.comports()) or "none")
        return
    m = Msp(sys.argv[1])
    time.sleep(0.2)
    try:
        what = sys.argv[2]
        if what == "info":
            info(m)
        elif info(m):
            if what == "rc":
                run_rc(m)
            elif what == "arm":
                run_arm(m)
            elif what == "motor":
                run_motor(m, int(sys.argv[3]), int(sys.argv[4]))
    except KeyboardInterrupt:
        print("stopped")
    finally:
        m.safe()
        m.ser.close()


if __name__ == "__main__":
    main()
