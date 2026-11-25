# DroneLink - one drone over WiFi (UDP to the M7, port 4210; the M7 passes it on to the M4).
# Its own thread: receives everything into the World, and every 50 ms sends the PING (no PING for
# 0.5 s -> the drone lands) and the trajectory, 1 s ahead. Only while the GUI runs: a frozen
# app stops the PING, so the drone does not fly on with nobody watching.
#
#   link = DroneLink(world, world.addDrone("192.168.1.50"), recorder)
#   link.command("CALIBRATE")           # HOLD LAND KILL START PING CALIBRATE (msg::FlightCmd, CMD_ dropped)
#   link.startTrajectory(traj)          # planner.Trajectory, relative to the hover point
#   link.close()

import socket
import threading
import time

import numpy as np

from communication.protocol import MSG, packet, parse
from communication.recorder import IN, OUT

PORT = 4210
TICK_S = 0.05                           # PING + trajectory every 50 ms
TRAJ_DT = 0.2                           # a trajectory point every 0.2 s...
TRAJ_POINTS = MSG.value("SEG_MAX_POINTS")   # ...6 per segment: 1 s ahead (repeats are fine: the drone replaces them)
TRAJ_DELAY = 0.5                        # s: the drone starts this long after TrajStart (it arrives 3 times)
FLY = 4                                 # ctrl::StateMachine::FLY (checked against StateMachine.h below)

TELEM = MSG.structs["HoverTelem"]
SEG, TRAJ_START, POINT = MSG.structs["SegHeader"], MSG.structs["TrajStart"], MSG.structs["TrajPoint"]
CMD = MSG.structs["FlightCommand"]
ROWS, FLOW, PT = MSG.structs["FrameRows"], MSG.structs["FlowHeader"], MSG.structs["PointMsg"]
DEPTH_HDR, DEPTH_PT = MSG.structs["PointDepthHeader"], MSG.structs["PointDepthMsg"]
PKT = {name: MSG.value(name) for name in ("PKT_HOVER_TELEM", "PKT_LOG", "PKT_SEG", "PKT_SEG_ACK", "PKT_SEG_NACK",
                                          "PKT_TRAJ_START", "PKT_FLIGHT_CMD", "PKT_FRAME_ROWS", "PKT_FLOW",
                                          "PKT_POINT_DEPTH")}
NACK = {v: k for k, v in MSG.enums["NackReason"].items()}


class DroneLink:
    def __init__(self, world, drone, recorder):
        from communication.protocol import SM
        assert SM.enums["State"]["FLY"] == FLY
        self.world, self.drone, self.rec = world, drone, recorder
        self.addr = (drone.ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)   # Windows default 8 KB: image slices got lost
        self.sock.settimeout(0.01)
        self.lock = threading.Lock()    # the trajectory: set by the GUI, sent by this thread
        self.traj, self.trajT0, self.segSeq = None, None, 0
        self.lastNack = None
        self.alive = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- sending
    def send(self, ptype, payload, times=1):
        for _ in range(times):
            try:
                self.sock.sendto(packet(ptype, payload), self.addr)
            except OSError:
                pass
        self.rec.packet(self.drone, OUT, ptype, payload)

    def command(self, name):            # HOLD / LAND / KILL / START / CALIBRATE: 3 times (UDP may lose one)
        if name in ("HOLD", "LAND", "KILL"):
            self.stopTrajectory()
        self.send(PKT["PKT_FLIGHT_CMD"], CMD.pack(MSG.value("CMD_" + name)), 1 if name == "PING" else 3)

    def startTrajectory(self, traj):    # flown from TRAJ_DELAY s from now, sent until stopped
        with self.lock:
            d = self.drone
            d.trajId = (d.trajId + 1) & 0xFFFF
            self.traj, self.trajT0 = traj, time.time() + TRAJ_DELAY
            d.trajOn = True
        self.send(PKT["PKT_TRAJ_START"], TRAJ_START.pack(d.trajId, int(TRAJ_DELAY * 1000)), 3)

    def stopTrajectory(self):           # stop sending (the drone holds where the last point was)
        with self.lock:
            self.traj, self.trajT0 = None, None
            self.drone.trajOn = False

    def sendSegment(self):              # the points from just before now to 1 s ahead
        with self.lock:
            if self.traj is None:
                return
            traj, t = self.traj, time.time() - self.trajT0
            seq, self.segSeq = self.segSeq, (self.segSeq + 1) & 0xFFFF
        k0 = max(0, int(t / TRAJ_DT))
        pts = b"".join(POINT.pack(round(k * TRAJ_DT * 1000), *traj.at(k * TRAJ_DT)) for k in range(k0, k0 + TRAJ_POINTS))
        self.send(PKT["PKT_SEG"], SEG.pack(self.drone.trajId, seq, TRAJ_POINTS) + pts)

    # ---------------------------------------------------------------- receiving
    def onTelemetry(self, p):
        values = TELEM.st.unpack(p)
        r = dict(zip(TELEM.fields, values))
        d = self.drone
        with self.world.lock:
            d.telem, d.telemTime = r, time.time()
            d.trail.append((r["x"], r["y"], r["z"]))
            d.history.append((d.telemTime, values))
            if int(r["state"]) == FLY:        # the drone's own origin, every packet: it moves when x / y control
                d.origin = (r["ox"], r["oy"], r["oz"])   # comes back after a camera loss (read once, the next
            elif d.origin is not None:                   # go-to started from the old point: log 24.11 14:44)
                d.origin = None               # left FLY: the drone forgot its trajectory, so do we
                self.stopTrajectory()
        self.rec.telemetry(d, values)

    def onPacket(self, ptype, p):
        d = self.drone
        d.packets += 1
        self.rec.packet(d, IN, ptype, p)
        if ptype == PKT["PKT_HOVER_TELEM"] and len(p) == TELEM.size:
            self.onTelemetry(p)
        elif ptype == PKT["PKT_FRAME_ROWS"] and len(p) >= ROWS.size:
            self.onRows(p)
        elif ptype == PKT["PKT_FLOW"] and len(p) >= FLOW.size:
            self.onFlow(p)
        elif ptype == PKT["PKT_POINT_DEPTH"] and len(p) >= DEPTH_HDR.size:
            n = DEPTH_HDR.unpack(p)["count"]
            if len(p) == DEPTH_HDR.size + n * DEPTH_PT.size:
                pts = [DEPTH_PT.st.unpack_from(p, DEPTH_HDR.size + i * DEPTH_PT.size) for i in range(n)]
                with self.world.lock:
                    d.depth = [(i, x / 1000, y / 1000, z / 1000, sd / 1000) for i, x, y, z, sd in pts]
                    d.depthTime = time.time()
        elif ptype == PKT["PKT_LOG"]:
            self.world.log(p.decode(errors="replace").rstrip(), d)
        elif ptype == PKT["PKT_SEG_ACK"]:
            d.ack = MSG.structs["SegAck"].unpack(p)
        elif ptype == PKT["PKT_SEG_NACK"]:
            d.nack = MSG.structs["SegNack"].unpack(p)
            if d.nack["reason"] != self.lastNack:     # once per reason, not 20 times a second
                self.lastNack = d.nack["reason"]
                self.world.log(f"segment refused: {NACK.get(d.nack['reason'], d.nack['reason'])}", d)

    def onRows(self, p):                # a slice of the camera image (one datagram holds a few rows)
        h = ROWS.unpack(p)
        w, rows, n = h["width"], h["height"], h["nRows"]
        if len(p) != ROWS.size + n * w or h["row0"] + n > rows:
            return
        d = self.drone
        with self.world.lock:
            if d.image is None or d.image.shape != (rows, w):
                d.image = np.zeros((rows, w), np.uint8)
            d.image[h["row0"]:h["row0"] + n] = np.frombuffer(p, np.uint8, n * w, ROWS.size).reshape(n, w)
            d.imageSeq, d.slices = h["seq"], d.slices + 1

    def onFlow(self, p):                # the tracked points of a frame pair
        h = FLOW.unpack(p)
        if len(p) != FLOW.size + h["count"] * PT.size:
            return
        d = self.drone
        pts = [PT.st.unpack_from(p, FLOW.size + i * PT.size) for i in range(h["count"])]
        with self.world.lock:
            d.flow, d.points, d.flows = h, pts, d.flows + 1
            d.flowHistory.append((time.time(), h["nTracked"], h["nInliers"], h["rmsPx"], h["valid"]))

    # ---------------------------------------------------------------- the thread
    def run(self):
        nextTick = time.time()
        while self.alive:
            try:
                data, _ = self.sock.recvfrom(8192)
                for ptype, p in parse(data):
                    self.onPacket(ptype, p)
            except socket.timeout:
                pass
            except OSError:
                time.sleep(0.01)
            if time.time() >= nextTick:
                nextTick = time.time() + TICK_S
                if self.world.guiAlive():
                    self.command("PING")
                    self.sendSegment()

    def close(self):
        self.alive = False
        self.thread.join(1.0)
        self.sock.close()
