# controlApp - the PC side of the swarm: connects to the drones, shows them, sends them commands
# and trajectories. The drones fly themselves (M4: trajTest.ino, M7: m7Runner with PC_WIFI 1).
#   python main.py NICLA_IP [NICLA_IP ...]      one window for all of them
#   python main.py --images NICLA_IP            also record the camera images (big)
#
# Folders:
#   communication/  the drone link (UDP), the packets (read from whoopCTRL/shared/headers/messages.h), the recorder
#   planner/        trajectories (relative to the hover point)
#   GUI/            the window: map (click = go there), 3D, camera, attitude, plots, drone panel, log
#   utilities/      helpers (cHeaders.py: reads the drone's C++ headers)
#   world.py        everything the app knows (drones; later obstacles, formation), shared by all of the above
#   logs/           session_*.bin (every packet) + session_*_droneN.csv (telemetry, hoverTest.py columns)
#
# Safety: closing the window = KILL ALL; Space = KILL ALL anytime; the app frozen > 1 s = no PING =
# the drones land by themselves.

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pyqtgraph as pg
from PySide6.QtWidgets import QApplication

from communication.droneLink import DroneLink
from communication.recorder import Recorder
from GUI.mainWindow import MainWindow
from world import World


def main():
    ap = argparse.ArgumentParser(description="controlApp: the drones' ground station")
    ap.add_argument("ips", nargs="+", help="each drone's Nicla IP")
    ap.add_argument("--images", action="store_true", help="record the camera images too")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    pg.setConfigOptions(imageAxisOrder="row-major", antialias=True)
    world = World()
    rec = Recorder(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"), images=args.images)
    links = [DroneLink(world, world.addDrone(ip), rec) for ip in args.ips]
    win = MainWindow(world, links, rec)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
