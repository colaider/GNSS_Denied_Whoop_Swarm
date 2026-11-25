// Flight sketch: take off, hover, then fly the trajectories the PC sends (controlApp: map click = go there,
// height box = climb / sink there). The M7 runs m7Runner (PC_WIFI 1).

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
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/visula/pointDepth.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/FILTERS/KF.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/DrCTRL/betaflight.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/CTRL/PosCTRL.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/CTRL/StateMachine.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/CTRL/Trajectory.cpp"

const sensors::Pose HOVER{0.0f, 0.0f, 0.60f, 0.0f};   // where to hover, start frame (calibration point on the floor = 0)

drone::BetaflightConfig fcConfig() {
  drone::BetaflightConfig c;
  c.channels = 18;
  c.rcRateHz = 100;                     // needs BF_BAUD 460800 (both sides): Betaflight smoothing 19 -> 37 Hz
  c.armLowMs = 1000;
  c.armTimeoutMs = 4000;
  return c;
}
ctrl::SMConfig smConfig() {
  ctrl::SMConfig c;
  c.climbRate = 0.5f;
  c.xyAfterS = 0.3f;
  c.descendRate = 0.1f;
  return c;
}

comm::CoreLink      core;
comm::Communication m7;
sensors::Sensors    sens;
drone::Betaflight   fc(fcConfig());
ctrl::PosCTRL       pos;              // gains: PosConfig in CTRL/headers/PosCTRL.h
ctrl::StateMachine  sm(sens, fc, pos, smConfig());
ctrl::Trajectory    traj;
sensors::PointDepth depth;              // watching only: the points' 3D positions for the PC

void say(const char* fmt, double a = 0, double b = 0, double c = 0) {
  char line[128] = "TRAJ: ";
  snprintf(line + 6, sizeof(line) - 6, fmt, a, b, c);
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
    else say(sm.calibrated() && !sm.referenceOk() ? "START ignored: FC not answering, no roll / pitch reference yet" : "START ignored: press C first (calibrate)");
  }
  else if (c == msg::CMD_HOLD) { sm.heard(); traj.clear(); }   // no more points: the loop holds the last one
  else if (c == msg::CMD_LAND) sm.land();
  else if (c == msg::CMD_KILL) sm.kill();
  else sm.heard();
}

void onStart(const msg::TrajStart& s) {
  sm.heard();
  if (sm.state() != ctrl::StateMachine::FLY) { say("trajectory ignored: not hovering yet (FLY)"); return; }
  if (traj.start(s.trajId, shared::nowUs() + s.delayMs * 1000UL)) say("trajectory %.0f", s.trajId);
}

void setGains(const msg::Gains& g) {    // autoTune.py, in flight. Out of 0..15 or NaN: refused
  for (int i = 0; i < 4; i++)
    if (!(g.kp[i] >= 0.0f && g.kp[i] <= 15.0f && g.kd[i] >= 0.0f && g.kd[i] <= 15.0f && g.ki[i] >= 0.0f && g.ki[i] <= 15.0f)) return;
  ctrl::PosConfig& c = pos.config();
  const bool same = c.kp.x == g.kp[0] && c.kp.y == g.kp[1] && c.kp.z == g.kp[2] && c.kd.x == g.kd[0] && c.kd.y == g.kd[1] && c.kd.z == g.kd[2];
  c.kp = sensors::Pose{g.kp[0], g.kp[1], g.kp[2], g.kp[3]};
  c.kd = sensors::Pose{g.kd[0], g.kd[1], g.kd[2], g.kd[3]};
  c.ki = sensors::Pose{g.ki[0], g.ki[1], g.ki[2], g.ki[3]};
  if (same) return;                     // the PC sends each 3 times
  say("gains kp %.2f %.2f %.2f", g.kp[0], g.kp[1], g.kp[2]);
  say("gains kd %.2f %.2f %.2f", g.kd[0], g.kd[1], g.kd[2]);
}

void readM7() {
  static comm::Packet p;
  bool gotFlow = false;
  while (m7.receive(p)) {
    if (p.type == msg::PKT_FLOW && p.len >= sizeof(msg::FlowHeader)) {
      const msg::FlowHeader& f = *(const msg::FlowHeader*)p.data;
      if (p.len != sizeof(msg::FlowHeader) + f.count * sizeof(msg::PointMsg)) continue;
      sens.update(&f, (const msg::PointMsg*)(p.data + sizeof(msg::FlowHeader)), f.count);
      depth.update(sens, f, (const msg::PointMsg*)(p.data + sizeof(msg::FlowHeader)), f.count);
      gotFlow = true;
    }
    else if (p.type == msg::PKT_SEG && p.len >= sizeof(msg::SegHeader)) {
      msg::SegHeader h;
      memcpy(&h, p.data, sizeof(h));
      if (h.n <= msg::SEG_MAX_POINTS && p.len == sizeof(h) + h.n * sizeof(msg::TrajPoint)) {
        msg::TrajPoint pts[msg::SEG_MAX_POINTS];
        memcpy(pts, p.data + sizeof(h), h.n * sizeof(msg::TrajPoint));
        traj.add(h.trajId, pts, h.n);
      }
    }
    else if (p.type == msg::PKT_TRAJ_START && p.len == sizeof(msg::TrajStart)) {
      msg::TrajStart s;
      memcpy(&s, p.data, sizeof(s));
      onStart(s);
    }
    else if (p.type == msg::PKT_GAINS && p.len == sizeof(msg::Gains)) {
      msg::Gains g;
      memcpy(&g, p.data, sizeof(g));
      sm.heard();
      setGains(g);
    }
    else if (p.type == msg::PKT_FLIGHT_CMD && p.len == sizeof(msg::FlightCommand)) onCommand(p.data[0]);
  }
  if (!gotFlow) sens.update();
}

void sendDepth() {                      // the surest points first, as many as fit one WiFi datagram
  static uint8_t buf[sizeof(msg::PointDepthHeader) + msg::POINT_DEPTH_MAX * sizeof(msg::PointDepthMsg)];
  int order[sensors::PointDepth::MAXP];
  const int n = depth.count();
  for (int i = 0; i < n; i++) {                       // insertion sort by std
    int k = i;
    while (k > 0 && depth.point(order[k - 1]).std > depth.point(i).std) { order[k] = order[k - 1]; k--; }
    order[k] = i;
  }
  msg::PointDepthHeader h;
  h.tUs = shared::nowUs();
  h.count = (uint8_t)(n < msg::POINT_DEPTH_MAX ? n : msg::POINT_DEPTH_MAX);
  memcpy(buf, &h, sizeof(h));
  auto mm = [](float m) { const float v = m * 1000.0f; return (int16_t)(v > 32000.0f ? 32000.0f : (v < -32000.0f ? -32000.0f : v)); };
  for (int k = 0; k < h.count; k++) {
    const sensors::PointDepth::Point& p = depth.point(order[k]);
    msg::PointDepthMsg o;
    o.id = p.id;
    o.x = mm(p.world.x);  o.y = mm(p.world.y);  o.z = mm(p.world.z);
    o.std = (uint16_t)(p.std * 1000.0f < 65000.0f ? p.std * 1000.0f : 65000.0f);
    memcpy(buf + sizeof(h) + k * sizeof(o), &o, sizeof(o));
  }
  m7.send(msg::PKT_POINT_DEPTH, buf, sizeof(h) + h.count * sizeof(msg::PointDepthMsg));
}

void sendTelem() {
  const sensors::Pose p = sens.pose(), v = sens.poseVelocity(), t = sm.setpoint(), i = pos.integral();
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
  const sensors::ToFReading& tr = sens.tofReading();
  m.tofRange = tr.rangeM;  m.tofHeight = sens.height();  m.tofSignal = tr.signalMcps;
  m.tofValid = tr.valid;  m.tofStatus = tr.status;  m.floorSteps = (uint16_t)sens.vertical().steps();
  m.camYawRate = sens.cameraDebug().yawRate * 57.29578f;  m.gyroYawRate = sens.cameraDebug().gyroYawRate * 57.29578f;
  const sensors::Pose o = sm.origin();
  m.ox = o.x;  m.oy = o.y;  m.oz = o.z;
  m7.send(msg::PKT_HOVER_TELEM, &m, sizeof(m));
}

void logChanges() {
  static bool link = false, xy = false;
  static uint8_t state = 255;
  char line[96];
  if (sm.xyControl() != xy) {
    xy = sm.xyControl();
    say(xy ? "camera tracking: x / y held from here" : "camera lost: level only, x / y drift");
  }
  if (fc.linkOk() != link) {
    link = fc.linkOk();
    say(link ? "FC answering" : "FC NOT answering");
  }
  if (sm.state() == state) return;
  if (state == ctrl::StateMachine::SPINUP && !fc.armed()) {
    snprintf(line, sizeof(line), "TRAJ: arming refused, blockers 0x%lx", (unsigned long)fc.armBlockers());
    m7.log(line);
  }
  state = sm.state();
  snprintf(line, sizeof(line), "TRAJ: %s  z %.3f m  battery %.2f V", sm.stateName(), sens.pose().z, fc.batteryV());
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

  // up to HOVER; then the trajectory's point for now, or (no point: not started, ran out, HOLD) the last one,
  // standing still. Kept relative to the origin: it moves when x / y control comes back after a camera loss.
  static sensors::Pose rel;                       // the last point, relative to sm.origin()
  const uint32_t now = shared::nowUs();
  sensors::Pose p, v, a;
  if (sm.state() != ctrl::StateMachine::FLY) { traj.clear();  rel = sensors::Pose();  sm.setTarget(HOVER); }
  else if (traj.sample(now, p, v, a)) { rel = p;  sm.setTarget(sm.origin() + p, v, a); }
  else sm.setTarget(sm.origin() + rel);
  sm.step(now);
  logChanges();

  static uint32_t telemUs = 0;
  if (shared::nowUs() - telemUs >= 20000) {
    telemUs = shared::nowUs();
    sendTelem();
    static uint8_t k = 0;
    if (++k % 5 == 0) sendDepth();      // 10 Hz
  }
}
