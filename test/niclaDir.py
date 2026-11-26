# Where a Nicla's calibration lives: whoopCTRL/drones/<name>_<uid>/ (or just <uid>/).
# <uid> = the chip's 24-hex ID, printed by imu_calib.ino, camera_calib.ino, niclaId.ino and the flight
# firmware's boot log. gen_calib_headers.py turns every folder into one CAL_<name> of drones/calibration.h.

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
NICLAS = os.path.normpath(os.path.join(HERE, "..", "whoopCTRL", "drones"))
UID_RE = re.compile(r"^(?:(.+)_)?([0-9a-f]{24})$")


def parse(folder_name):
    """(name, uid) from a folder name, None if it has no UID."""
    m = UID_RE.match(folder_name)
    return (m.group(1) or "", m.group(2)) if m else None


def folder(uid):
    """The folder of this UID (made as drones/<uid>/ if there is none yet: rename it to <name>_<uid>)."""
    uid = uid.lower()
    if not re.fullmatch(r"[0-9a-f]{24}", uid):
        raise SystemExit(f"bad Nicla UID: {uid!r}")
    os.makedirs(NICLAS, exist_ok=True)
    for d in os.listdir(NICLAS):
        p = parse(d)
        if p and p[1] == uid and os.path.isdir(os.path.join(NICLAS, d)):
            return os.path.join(NICLAS, d)
    path = os.path.join(NICLAS, uid)
    os.makedirs(path)
    print(f"new Nicla {uid}: made {path}  (rename it to e.g. drone2_{uid})")
    return path
