# World - everything the app knows, in one place: the drones (what they report, where they were sent),
# later the obstacles and the formation. The communication writes it, the planner and the GUI read it.
# One lock: the drone links write from their own threads, the GUI reads ~20 times a second.
#
# Frames: "start frame" = the drone's own, x forward, y left, z up, (0, 0, 0) = where it was calibrated
# on the floor. Trajectories are relative to the hover point (StateMachine::origin), in the start frame.

import threading
import time
from collections import deque

GUI_STALE_S = 1.0                       # the GUI did not run this long: the links stop the PING -> the drones land


class Drone:
    def __init__(self, index, ip):
        self.index, self.ip, self.name = index, ip, f"drone {index}"
        self.telem = None               # the last HoverTelem, dict by field name (None: nothing heard yet)
        self.telemTime = 0.0            # PC time it came
        self.trail = deque(maxlen=3000) # (x, y, z), start frame
        self.origin = None              # the hover point in the start frame (None: not flying)
        self.goal = None                # where it was sent, start frame (x, y, z)
        self.path = []                  # the trajectory sent, start frame, [(x, y, z)], to draw
        self.trajId, self.trajOn = 0, False   # the trajectory being sent
        self.ack = self.nack = None     # the last SegAck / SegNack, dict
        self.packets = 0                # received
        self.history = deque(maxlen=3000)   # (PC time, HoverTelem values in field order): ~60 s at 50 Hz, for the plots
        self.image, self.imageSeq = None, -1   # camera image (numpy rows x cols, uint8), from the slices
        self.slices = 0                 # image slices received
        self.flow, self.points = None, []   # the last FlowHeader (dict) and its tracked points (PointMsg tuples)
        self.flows = 0                  # flow packets received
        self.flowHistory = deque(maxlen=1800)   # (PC time, tracked, inliers, rmsPx, valid)
        self.depth, self.depthTime = [], 0.0    # PointDepth: [(id, x, y, z, std)] m, start frame, surest first

    def alive(self, now=None):          # telemetry within the last second
        return self.telem is not None and (now or time.time()) - self.telemTime < 1.0

    def state(self):
        from communication.protocol import STATES
        return STATES.get(int(self.telem["state"]), "?") if self.telem else "?"


class World:
    def __init__(self):
        self.lock = threading.RLock()
        self.drones = []
        self.obstacles = []             # later: polygons, start frame
        self.logs = deque(maxlen=1000)  # (PC time, text)
        self.logCount = 0               # ever added (the GUI shows the new ones)
        self.guiBeat = time.time()      # the GUI's last tick: the links ping the drones only while it runs

    def addDrone(self, ip):
        with self.lock:
            d = Drone(len(self.drones), ip)
            self.drones.append(d)
            return d

    def log(self, text, drone=None):
        line = time.strftime("%H:%M:%S ") + (f"[{drone.name}] " if drone else "") + text
        with self.lock:
            self.logs.append(line)
            self.logCount += 1
        print(line)

    def guiAlive(self):
        return time.time() - self.guiBeat < GUI_STALE_S
