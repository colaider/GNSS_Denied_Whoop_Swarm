# MainWindow - map (click = go there) and 3D in the middle; around them docks: drones (status, commands),
# camera, attitude, plots, log. Every dock can be moved, floated, closed (menu Panels: back again);
# the layout is remembered. Turns clicks and keys into commands for the drone links; no control here:
# the drones fly themselves, the app only says where to.

import time

from PySide6.QtCore import QEvent, QSettings, Qt, QTimer
from PySide6.QtWidgets import QAbstractSpinBox, QApplication, QDockWidget, QLineEdit, QMainWindow, QSplitter

from GUI.attitudeView import AttitudeView
from GUI.cameraView import CameraView
from GUI.dronePanel import DronePanel
from GUI.logView import LogView
from GUI.mapView import MapView
from GUI.plotsView import PlotsView
from GUI.view3d import View3D
from planner.trajectory import Trajectory, clampGoal

KEYS = {Qt.Key_C: "CALIBRATE", Qt.Key_Return: "START", Qt.Key_Enter: "START", Qt.Key_H: "HOLD", Qt.Key_L: "LAND"}


class MainWindow(QMainWindow):
    def __init__(self, world, links, recorder):
        super().__init__()
        self.world, self.links, self.rec = world, links, recorder
        self.setWindowTitle("controlApp  " + "  ".join(d.ip for d in world.drones))
        self.resize(1600, 1000)

        self.map = MapView()
        self.map.clicked.connect(self.goTo)
        self.view3d = View3D()
        centre = QSplitter(Qt.Horizontal)
        centre.addWidget(self.map)
        centre.addWidget(self.view3d)
        self.setCentralWidget(centre)
        self.panel = DronePanel(world)
        self.panel.command.connect(self.command)
        self.panel.height.valueChanged.connect(self.changeHeight)
        self.camera, self.attitude = CameraView(), AttitudeView()
        self.plots, self.log = PlotsView(), LogView(world)
        menu = self.menuBar().addMenu("Panels")
        docks = {}
        for title, widget, side in (("drones", self.panel, Qt.RightDockWidgetArea),
                                    ("camera", self.camera, Qt.LeftDockWidgetArea),
                                    ("attitude", self.attitude, Qt.LeftDockWidgetArea),
                                    ("plots", self.plots, Qt.BottomDockWidgetArea),
                                    ("log", self.log, Qt.BottomDockWidgetArea)):
            dock = QDockWidget(title)
            dock.setObjectName(title)           # the saved layout finds it by this
            dock.setWidget(widget)
            self.addDockWidget(side, dock)
            menu.addAction(dock.toggleViewAction())
            docks[title] = dock
        self.tabifyDockWidget(docks["plots"], docks["log"])
        docks["plots"].raise_()
        self.settings = QSettings("whoopSwarm", "controlApp")
        self.resizeDocks([docks["camera"], docks["drones"]], [420, 380], Qt.Horizontal)   # first start; later
        self.resizeDocks([docks["plots"]], [360], Qt.Vertical)                              # the saved layout
        self.restoreGeometry(self.settings.value("geometry", b""))
        self.restoreState(self.settings.value("state", b""))
        self.ticks = 0

        QApplication.instance().installEventFilter(self)   # the keys work wherever the focus is
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        world.log(f"session -> {recorder.base}.bin")

    def tick(self):
        self.world.guiBeat = time.time()      # the links ping the drones only while this runs
        d = self.panel.selected()
        self.map.refresh(self.world, d)
        self.view3d.refresh(self.world, d)
        self.panel.refresh()
        self.log.refresh()
        for view in (self.camera, self.attitude, self.plots):     # hidden docks cost nothing
            if view.isVisible() and (view is not self.plots or self.ticks % 2 == 0):   # plots: 10 Hz
                view.refresh(self.world, d)
        self.ticks += 1

    # ---------------------------------------------------------------- commands
    def command(self, name):
        if name == "KILL":                    # every drone
            for link in self.links:
                link.command("KILL")
            self.world.log("PC: KILL ALL")
            return
        d = self.panel.selected()
        if d is None:
            return
        if name == "CALIBRATE":               # a new start frame: the old marks mean nothing now
            with self.world.lock:
                d.trail.clear()
                d.goal, d.path = None, []
        if name in ("HOLD", "LAND"):
            with self.world.lock:
                d.path = []
        self.links[d.index].command(name)
        self.world.log(f"PC: {name}", d)

    def goTo(self, x, y):                     # map click: the selected drone flies there (hovering only)
        d = self.panel.selected()
        if d is None:
            return
        with self.world.lock:
            if not d.alive() or d.state() != "FLY" or d.origin is None:
                self.world.log("not hovering: calibrate (C) and take off (Enter) first", d)
                return
            o, r = d.origin, d.telem
            goal = clampGoal((x, y, self.panel.height.value()), o)
            start = (r["tx"] - o[0], r["ty"] - o[1], r["tz"] - o[2])   # where its setpoint is now
        traj = Trajectory.goTo(start, tuple(g - k for g, k in zip(goal, o)), self.panel.speed.value())
        self.links[d.index].startTrajectory(traj)
        with self.world.lock:
            d.goal = goal
            d.path = [tuple(p + k for p, k in zip(q, o)) for q in traj.points()]
        self.world.log(f"PC: go to x {goal[0] * 100:+.0f} y {goal[1] * 100:+.0f} z {goal[2] * 100:.0f} cm "
                       f"({traj.duration:.1f} s, trajectory {d.trajId})", d)

    def changeHeight(self, _):                # new go-to height: a hovering drone goes there, x / y as they are
        d = self.panel.selected()
        if d is None:
            return
        with self.world.lock:
            if not d.alive() or d.state() != "FLY" or d.origin is None:
                return                        # not flying: the height is used by the next take-off / map click
            x, y = d.telem["tx"], d.telem["ty"]   # its setpoint now
        self.goTo(x, y)

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.KeyPress and not ev.isAutoRepeat():
            if ev.key() == Qt.Key_Space:      # KILL ALL: always, whatever has the focus
                self.command("KILL")
                return True
            typing = isinstance(QApplication.focusWidget(), (QAbstractSpinBox, QLineEdit))
            if ev.key() in KEYS and not typing:
                self.command(KEYS[ev.key()])
                return True
        return super().eventFilter(obj, ev)

    def closeEvent(self, ev):                 # closing the app: motors off
        self.timer.stop()
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("state", self.saveState())
        for link in self.links:
            link.command("KILL")
            link.close()
        self.rec.close()
        print(f"saved {self.rec.base}.bin (+ _droneN.csv)")
        ev.accept()
