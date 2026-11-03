// Hover test. PC: python hoverTest.py NICLA_IP   C = calibrate, Enter = arm + hover, L = land, Space = kill.
// M7: m7Runner with PC_WIFI 1. Gains and limits: posConfig() / smConfig() below, then re-upload.

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#include <RPC.h>
#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/IMU.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/ToF.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/visula/visulaVel.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/SENSORS.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/FILTERS/KF.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/DrCTRL/betaflight.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/CTRL/PosCTRL.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/CTRL/StateMachine.cpp"

const float MAX_HEIGHT = 3.0f;          // m: higher -> kill

drone::BetaflightConfig fcConfig() {
  drone::BetaflightConfig c;
  c.channels = 18;
  c.armLowMs = 1000;                    // the FC lost RC during the 2 s calibration
  c.armTimeoutMs = 4000;
  return c;
}
ctrl::PosConfig posConfig() {
  ctrl::PosConfig c;
     c.kp = sensors::Pose{4.0f, 4.0f, 9.0f, 0.0f};
  c.kd = sensors::Pose{3.2f, 3.2f, 4.8f, 0.0f};
  c.ki = sensors::Pose{1.0f, 1.0f, 1.5f, 0.0f};
  c.throttleHover = 0.32f;
  c.maxTiltDeg = 8.0f;
  // c.throttleHover = 0.30;
  c.throttleMax = 0.45f;
  c.maxYawRateDps = 20.0f;
  // c.pitchSign = -1.0f;
  // c.rollSign  = 1.0f;
  return c;
}
ctrl::SMConfig smConfig() {
  ctrl::SMConfig c;
  c.hoverHeight = 0.40f;
  c.reachedM = 0.02f;
  c.descendRate = 0.1f;
  c.safetyRate = 0.1f;
  return c;
}

comm::CoreLink      core;
comm::Communication m7;
sensors::Sensors    sens;
drone::Betaflight   fc(fcConfig());
ctrl::PosCTRL       pos(posConfig());
ctrl::StateMachine  sm(sens, fc, pos, smConfig());

void say(const char* fmt, double a = 0, double b = 0, double c = 0) {
  char line[128] = "HOVER: ";
  snprintf(line + 7, sizeof(line) - 7, fmt, a, b, c);
  m7.log(line);
}

void flush() {                          // packets that waited while we were blocked are old
  static comm::Packet old;
  while (m7.receive(old)) {}
}

void onCommand(uint8_t c) {
  if (sm.state() != ctrl::StateMachine::READY && (c == msg::CMD_CALIBRATE || c == msg::CMD_START)) sm.heard();
  else if (c == msg::CMD_CALIBRATE) {
    say("calibrating: keep it STILL");
    say(sm.calibrate() ? "calibrated: position 0 here. Enter = take off" : "moved during calibration, press C again");
    flush();
  }
  else if (c == msg::CMD_START) {
    if (sm.start()) say("arming");
    else say("START ignored: press C first (calibrate)");
  }
  else if (c == msg::CMD_LAND) sm.land();
  else if (c == msg::CMD_KILL) sm.kill();
  else sm.heard();
}

void readM7() {
  static comm::Packet p;
  bool gotFlow = false;
  while (m7.receive(p)) {
    if (p.type == msg::PKT_FLOW && p.len >= sizeof(msg::FlowHeader)) {
      const msg::FlowHeader& f = *(const msg::FlowHeader*)p.data;
      if (p.len != sizeof(msg::FlowHeader) + f.count * sizeof(msg::PointMsg)) continue;
      sens.update(&f, (const msg::PointMsg*)(p.data + sizeof(msg::FlowHeader)), f.count);
      gotFlow = true;
    }
    else if (p.type == msg::PKT_FLIGHT_CMD && p.len == sizeof(msg::FlightCommand)) onCommand(p.data[0]);
  }
  if (!gotFlow) sens.update();
}

void sendTelem() {
  const sensors::Pose p = sens.pose(), v = sens.poseVelocity(), t = sm.target(), i = pos.integral();
  const ctrl::PosCTRL::Command& c = sm.command();
  const sensors::Vec3 a = sens.imuAcceleration();
  const sensors::Attitude at = sens.attitude();
  msg::HoverTelem m;
  m.tUs = shared::nowUs();
  m.state = sm.state();
  m.flags = (fc.armed() ? msg::HOV_ARMED : 0) | (fc.linkOk() ? msg::HOV_FC_LINK : 0) | (sens.horizontalFix() != sensors::Fix::LOST ? msg::HOV_CAM_OK : 0) | (sens.verticalFix() != sensors::Fix::LOST ? msg::HOV_TOF_OK : 0) | (sens.imu().ready() ? msg::HOV_CALIBRATED : 0);
  m.x = p.x;  m.y = p.y;  m.z = p.z;  m.yaw = p.yaw;
  m.vx = v.x; m.vy = v.y; m.vz = v.z; m.vyaw = v.yaw;
  m.tx = t.x; m.ty = t.y; m.tz = t.z; m.tyaw = t.yaw;
  m.roll = c.rollDeg;  m.pitch = c.pitchDeg;  m.yawRate = c.yawRateDps;  m.throttle = c.throttle;
  m.ix = i.x; m.iy = i.y; m.iz = i.z;
  m.ax = a.x; m.ay = a.y; m.az = a.z;
  m.attRoll = at.roll * 57.29578f;  m.attPitch = at.pitch * 57.29578f;
  m.battery = fc.batteryV();
  m7.send(msg::PKT_HOVER_TELEM, &m, sizeof(m));
}

void logChanges() {
  static bool link = false, xy = false;
  static uint8_t state = 255;
  if (sm.xyControl() != xy) {
    xy = sm.xyControl();
    say(xy ? "camera tracking: x / y held from here" : "camera lost: level only, x / y drift");
  }
  if (fc.linkOk() != link) {
    link = fc.linkOk();
    say(link ? "FC answering" : "FC NOT answering");
  }
  if (sm.state() == state) return;
  char line[96];
  if (state == ctrl::StateMachine::TAKEOFF && !fc.armed()) {
    snprintf(line, sizeof(line), "HOVER: arming refused, blockers 0x%lx", (unsigned long)fc.armBlockers());
    m7.log(line);
  }
  state = sm.state();
  snprintf(line, sizeof(line), "HOVER: %s  z %.3f m  battery %.2f V", sm.stateName(), sens.pose().z, fc.batteryV());
  m7.log(line);
}

void setup() {
  comm::addLink(core);
  m7.begin("core");
  while (!sm.begin()) {
    say("IMU or ToF not answering, retrying");
    delay(1000);
  }
  say("sensors ok");
}

void loop() {
  readM7();

  sm.step(shared::nowUs());
  logChanges();

  static uint32_t telemUs = 0;
  if (shared::nowUs() - telemUs >= 20000) {
    telemUs = shared::nowUs();
    sendTelem();
  }
}
