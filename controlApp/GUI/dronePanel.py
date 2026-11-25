# DronePanel - the drones (pick one), what the selected one reports, and the commands.
# The buttons only ask: the main window does the sending (signal command(name)).

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QGridLayout, QLabel, QListWidget, QPushButton,
                               QVBoxLayout, QWidget)

from communication.protocol import MSG
from GUI.mapView import colour
from planner.trajectory import MAX_HEIGHT, MAX_SPEED

FLAGS = MSG.enums["HoverFlags"]


class DronePanel(QWidget):
    command = Signal(str)               # CALIBRATE / START / HOLD / LAND / KILL

    def __init__(self, world):
        super().__init__()
        self.world = world
        lay = QVBoxLayout(self)

        self.list = QListWidget()
        self.list.setMaximumHeight(110)
        for d in world.drones:
            self.list.addItem(d.name)
            self.list.item(d.index).setForeground(QColor(colour(d.index)))
        self.list.setCurrentRow(0)
        lay.addWidget(self.list)

        self.status = QLabel()
        self.status.setFont(QFont("Consolas", 10))
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.status.setMinimumWidth(380)
        lay.addWidget(self.status)

        grid = QGridLayout()
        buttons = [("Calibrate  (C)", "CALIBRATE"), ("Take off  (Enter)", "START"),
                   ("Hold  (H)", "HOLD"), ("Land  (L)", "LAND")]
        for i, (text, name) in enumerate(buttons):
            b = QPushButton(text)
            b.setFocusPolicy(Qt.NoFocus)    # Space / Enter never press a button by accident
            b.clicked.connect(lambda _=False, n=name: self.command.emit(n))
            grid.addWidget(b, i // 2, i % 2)
        kill = QPushButton("KILL ALL  (Space)")
        kill.setFocusPolicy(Qt.NoFocus)
        kill.setStyleSheet("background:#dc2626; color:white; font-weight:bold; padding:10px;")
        kill.clicked.connect(lambda: self.command.emit("KILL"))
        grid.addWidget(kill, 2, 0, 1, 2)
        lay.addLayout(grid)

        form = QFormLayout()
        self.height = QDoubleSpinBox(minimum=0.1, maximum=MAX_HEIGHT, singleStep=0.1, value=0.6, suffix=" m")
        self.speed = QDoubleSpinBox(minimum=0.05, maximum=MAX_SPEED, singleStep=0.05, value=0.3, suffix=" m/s")
        for s in (self.height, self.speed):
            s.setFocusPolicy(Qt.ClickFocus)
            s.setKeyboardTracking(False)        # a typed value counts on Enter / leaving the box, not per key
        form.addRow("go-to height", self.height)
        form.addRow("go-to speed", self.speed)
        lay.addLayout(form)
        hint = QLabel("Click the map: the selected drone flies there (hovering only).\n"
                      "Change the height: it climbs / sinks there where it is.\n"
                      "C calibrate · Enter take off · H hold · L land · Space KILL ALL")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#666;")
        lay.addWidget(hint)
        lay.addStretch(1)

    def selected(self):
        i = self.list.currentRow()
        return self.world.drones[i] if 0 <= i < len(self.world.drones) else None

    def refresh(self):
        now = time.time()
        with self.world.lock:
            for d in self.world.drones:
                st = f"{d.state():<7} {d.telem['battery']:.2f} V" if d.alive(now) else "no telemetry"
                self.list.item(d.index).setText(f"{d.name}   {d.ip}   {st}")
            d = self.selected()
            self.status.setText(statusText(d, now) if d else "")


def statusText(d, now):
    if not d.alive(now):
        return (f"{d.name}  {d.ip}\n\nNO TELEMETRY\nsame WiFi? right IP? drone on (m7Runner PC_WIFI 1)?\n"
                f"packets received: {d.packets}")
    r = d.telem
    f = int(r["flags"])
    ok = lambda name: "ok" if f & FLAGS[name] else "--"
    lines = [f"{d.name}  {d.ip}",
             f"{d.state():<7} {'ARMED' if f & FLAGS['HOV_ARMED'] else 'disarmed':<9} {r['battery']:.2f} V",
             f"FC {ok('HOV_FC_LINK')}  camera {ok('HOV_CAM_OK')}  ToF {ok('HOV_TOF_OK')}  "
             f"{'calibrated' if f & FLAGS['HOV_CALIBRATED'] else 'NOT CALIBRATED (C)'}",
             "",
             f"x {r['x'] * 100:+7.1f}  y {r['y'] * 100:+7.1f}  z {r['z'] * 100:6.1f} cm",
             f"target  {r['tx'] * 100:+7.1f}    {r['ty'] * 100:+7.1f}    {r['tz'] * 100:6.1f}",
             f"speed  {r['vx'] * 100:+6.1f} {r['vy'] * 100:+6.1f} {r['vz'] * 100:+6.1f} cm/s",
             f"yaw {r['yaw'] * 57.2958:+6.1f} deg   roll {r['attRoll']:+5.1f}  pitch {r['attPitch']:+5.1f}",
             f"throttle {r['throttle']:.3f}   ToF {r['tofRange'] * 100:5.1f} cm  signal {r['tofSignal']:4.1f}",
             "",
             f"hover point  " + (f"{d.origin[0] * 100:+.0f} {d.origin[1] * 100:+.0f} {d.origin[2] * 100:.0f} cm"
                                 if d.origin else "- (not flying)"),
             f"trajectory {d.trajId}: {'sending' if d.trajOn else 'off'}" +
             (f"   queued {d.ack['queued']}  {d.ack['bufferedMs']} ms" if d.ack and d.trajOn else "")]
    return "\n".join(lines)
