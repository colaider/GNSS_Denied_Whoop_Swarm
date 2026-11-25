# Recorder - every packet of the session, both ways, to logs/ (git ignores it):
#   SESSION.bin           every packet raw: record = '<dBBBI' (PC time, drone, 0 in / 1 out, type, length) + payload.
#                         Replays the whole session later (images only with --images: they are big)
#   SESSION_droneN.csv    HoverTelem per drone, the columns of hoverTest.py's logs (same analysis scripts)

import csv
import os
import struct
import threading
import time

from communication.protocol import MSG

RECORD = struct.Struct("<dBBBI")
IN, OUT = 0, 1


class Recorder:
    def __init__(self, folder, images=False):
        os.makedirs(folder, exist_ok=True)
        self.base = os.path.join(folder, time.strftime("session_%Y%m%d_%H%M%S"))
        self.lock = threading.Lock()
        self.raw = open(self.base + ".bin", "wb")
        self.csvs = {}                  # drone index -> (file, writer)
        self.skip = set() if images else {MSG.value("PKT_FRAME_ROWS"), MSG.value("PKT_FRAME"), MSG.value("PKT_RAW")}
        self.telem = MSG.structs["HoverTelem"]

    def packet(self, drone, direction, ptype, payload):
        if ptype in self.skip:
            return
        with self.lock:
            if self.raw.closed:
                return
            self.raw.write(RECORD.pack(time.time(), drone.index, direction, ptype, len(payload)) + payload)

    def telemetry(self, drone, values):  # values: HoverTelem in field order
        with self.lock:
            if self.raw.closed:
                return
            if drone.index not in self.csvs:
                f = open(f"{self.base}_drone{drone.index}.csv", "w", newline="")
                w = csv.writer(f)
                w.writerow(self.telem.fields)
                self.csvs[drone.index] = (f, w)
            self.csvs[drone.index][1].writerow([f"{v:.5f}" if isinstance(v, float) else v for v in values])

    def close(self):
        with self.lock:
            self.raw.close()
            for f, _ in self.csvs.values():
                f.close()
