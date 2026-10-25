// M4 sensor test: IMU + ToF + camera flow (from the M7) -> position, sent to the PC.
//
// Runs with M7/m7Runner on the M7, which starts this core, sends it the flow of every frame
// (with all points) and passes our packets on to the PC. Every flow packet:
//   Sensors::update() -> camera velocity (derotated with the gyro) -> camera position
//                     -> Kalman filters (IMU + camera -> x, y; IMU + ToF -> z) -> fused pose
//   -> PKT_SENSORS to the PC: fused position / velocity / std / bias, the filters' innovations,
//      camera and IMU-only position / velocity, attitude, ToF, accelerometer
// Every 5 s a status line (PKT_LOG): loop / IMU / ToF / flow rates, flow delay, link errors,
// and how the filters took the measurements.
//
// Upload: Tools > Board: Arduino Nicla Vision, Target core: M4 Co-processor,
//         Flash split: "1MB M7 + 1MB M4" (the same split as the M7 sketch).
// Keep the drone STILL for the first seconds after reset: startup() measures the gyro bias
// and sets the world frame (yaw 0 = the way it faces now).
// View on the PC: python M4/test/pose_viewer.py (close Serial Monitor first).

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#include <RPC.h>       // listed here so the Arduino IDE finds the libraries
#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>

// The IDE only compiles this sketch folder, so the project code is included by absolute path.
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/IMU.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/ToF.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/visula/visulaVel.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/SENSORS/SENSORS.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/FILTERS/KF.cpp"

const uint32_t STATS_EVERY_MS = 5000;  // a status line to the PC

comm::CoreLink      core;
comm::Communication m7;                // to the M7 (and through it to the PC)
sensors::Sensors    sens;

// counters for the status line
uint32_t loops = 0, imuSamples = 0, tofReadings = 0, flows = 0, camValid = 0;
uint32_t flowDelaySumUs = 0, flowDelayMaxUs = 0;
uint32_t camApplied = 0, camRefused = 0, camLate = 0;   // what the horizontal filter did with the camera

void logLine(const char* fmt, long a = 0, long b = 0, long c = 0, long d = 0, long e = 0) {
  char line[128] = "M4: ";
  snprintf(line + 4, sizeof(line) - 4, fmt, a, b, c, d, e);
  m7.log(line);
}

void sendState(uint32_t flowSeq) {
  const sensors::Vec3 cp = sens.cameraPosition(), cv = sens.cameraVelocity();
  const sensors::Vec3 ip = sens.imuPosition(), iv = sens.imuVelocity();
  const sensors::Attitude a = sens.gyroPosition();
  const sensors::ToFReading& tof = sens.tofReading();
  msg::SensorState s;
  s.tUs     = shared::nowUs();
  s.camUs   = sens.cameraTimeUs();
  s.flowSeq = flowSeq;
  s.flags   = (sens.cameraValid() ? msg::SENS_CAM_VALID : 0) | (sens.imu().ready() ? msg::SENS_IMU_READY : 0) |
              (tof.valid ? msg::SENS_TOF_VALID : 0) | (sens.still() ? msg::SENS_STILL : 0) |
              ((uint8_t)sens.horizontalFix() << msg::SENS_HFIX_SHIFT) | ((uint8_t)sens.verticalFix() << msg::SENS_VFIX_SHIFT);
  s.camX = cp.x;  s.camY = cp.y;  s.camZ = cp.z;  s.camVx = cv.x;  s.camVy = cv.y;
  s.imuX = ip.x;  s.imuY = ip.y;  s.imuVx = iv.x; s.imuVy = iv.y;
  s.roll = a.roll; s.pitch = a.pitch; s.yaw = a.yaw;
  s.rangeM = tof.rangeM;
  const sensors::Vec3 acc = sens.imu().acceleration();
  s.accX = acc.x;  s.accY = acc.y;  s.accZ = acc.z;
  const sensors::Sensors::CameraDebug& d = sens.cameraDebug();
  s.rotX = d.rot.x;  s.rotY = d.rot.y;  s.rotZ = d.rot.z;
  s.dZ = d.dZ;
  s.camVxC = d.vx;  s.camVyC = d.vy;

  const sensors::Vec3 fp = sens.position(), fv = sens.velocity();
  const sensors::Vec3 sp = sens.positionStd(), sv = sens.velocityStd(), b = sens.accBias();
  const sensors::Vec3 aw = sens.imuAcceleration();
  s.fusX = fp.x;  s.fusY = fp.y;  s.fusZ = fp.z;  s.fusVx = fv.x;  s.fusVy = fv.y;  s.fusVz = fv.z;
  s.stdX = sp.x;  s.stdY = sp.y;  s.stdZ = sp.z;  s.stdVx = sv.x;  s.stdVy = sv.y;  s.stdVz = sv.z;
  s.biasX = b.x;  s.biasY = b.y;  s.biasZ = b.z;
  s.accWX = aw.x; s.accWY = aw.y; s.accWZ = aw.z;
  s.height = sens.height();
  const filters::HorizontalKF& h = sens.horizontal();
  const filters::VerticalKF& v = sens.vertical();
  const sensors::Sensors::FusionDebug& f = sens.fusionDebug();
  s.camInnovX = h.innovation(0);  s.camInnovY = h.innovation(1);
  s.camInnovStdX = h.innovationStd(0);  s.camInnovStdY = h.innovationStd(1);
  s.tofInnov = v.innovation();  s.tofInnovStd = v.innovationStd();
  s.camStd = f.camStd;  s.tofStd = f.tofStd;
  s.camResult = f.camResult;  s.tofResult = f.tofResult;
  s.camForced = (uint16_t)h.forced();  s.tofSteps = (uint16_t)v.steps();
  m7.send(msg::PKT_SENSORS, &s, sizeof(s));
}

void count(uint8_t got) {
  if (got & sensors::Sensors::NEW_IMU) imuSamples++;
  if (got & sensors::Sensors::NEW_TOF) tofReadings++;
}

void sendStats() {
  static uint32_t last = shared::nowUs();
  const uint32_t now = shared::nowUs();
  const uint32_t dt = now - last;
  if (dt < STATS_EVERY_MS * 1000UL) return;
  const uint32_t ms = dt / 1000;
  logLine("loop %ld/s, imu %ld/s (416 expected), tof %ld/s (restarts %ld), flow %ld/s",
          loops * 1000L / ms, imuSamples * 1000L / ms, tofReadings * 1000L / ms, sens.tof().restarts(), flows * 1000L / ms);
  logLine("camera valid %ld of %ld flow packets; filter took %ld, refused %ld, too late %ld",
          camValid, flows, camApplied, camRefused, camLate);
  logLine("filter: camera forced in %ld times, floor steps %ld",
          (long)sens.horizontal().forced(), (long)sens.vertical().steps());
  logLine("flow delay (frame end -> here) avg %ld us, max %ld us; link errors %ld, overflow bytes %ld",
          flows ? flowDelaySumUs / flows : 0, flowDelayMaxUs, m7.rxErrors(), core.rxOverflows());
  loops = imuSamples = tofReadings = flows = camValid = flowDelaySumUs = flowDelayMaxUs = 0;
  camApplied = camRefused = camLate = 0;
  last = now;
}

void setup() {
  comm::addLink(core);
  m7.begin("core");                      // connects to the M7 (it started us)
  logLine("sensor test starting");

  if (!sens.begin()) logLine("IMU or ToF did not answer - going on, check the values");
  while (!sens.startup(2000)) logLine("moved during startup - put it down on a table, trying again");
  logLine("startup ok: world frame set, positions 0. Move it now.");

  // flow packets that waited in the buffer during startup are seconds old: drop them
  static comm::Packet old;
  while (m7.receive(old)) {}
}

void loop() {
  loops++;
  static comm::Packet p;                 // static: 1.4 KB, kept off the stack
  bool gotFlow = false;

  while (m7.receive(p)) {
    if (p.type != msg::PKT_FLOW || p.len < sizeof(msg::FlowHeader)) continue;
    const msg::FlowHeader& flow = *(const msg::FlowHeader*)p.data;
    const int n = flow.count;
    if (p.len != sizeof(msg::FlowHeader) + n * sizeof(msg::PointMsg)) continue;   // count and length disagree

    const uint32_t delay = shared::nowUs() - flow.t1Us;   // also checks the shared clock: small and positive
    flowDelaySumUs += delay;
    if (delay > flowDelayMaxUs) flowDelayMaxUs = delay;
    flows++;

    const msg::PointMsg* pts = (const msg::PointMsg*)(p.data + sizeof(msg::FlowHeader));
    count(sens.update(&flow, pts, n));
    if (sens.cameraValid()) {
      camValid++;
      const int8_t r = sens.fusionDebug().camResult;
      if (r > 0) camApplied++; else if (r == 0) camRefused++; else camLate++;   // 3 (reset) counts as taken
    }
    sendState(flow.seq);
    gotFlow = true;
  }
  if (!gotFlow) count(sens.update());

  sendStats();
}
