// Kalman filter calibration: sends the filters' inputs to the PC (every IMU sample, ToF reading
// and camera velocity) and what the filters did with them. The PC runs the filters again offline
// and fits their noise values.
// PC: python kfCalib.py NICLA_IP   (it says what to do: still / slow / fast, C = calibrate)
// M7: m7Runner with PC_WIFI 1. No flight controller needed.

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

comm::CoreLink      core;
comm::Communication m7;
sensors::Sensors    sens;

msg::CalImuSample imuBuf[msg::CAL_IMU_PER_PACKET];
int imuN = 0;
uint32_t nImu = 0, nTof = 0, nCam = 0;

void say(const char* fmt, double a = 0, double b = 0, double c = 0) {
  char line[128] = "CAL: ";
  snprintf(line + 5, sizeof(line) - 5, fmt, a, b, c);
  m7.log(line);
}

void flush() {                          // packets that waited while we were blocked are old
  static comm::Packet old;
  while (m7.receive(old)) {}
}

void calibrate() {
  say("calibrating: keep it STILL");
  say(sens.startup() ? "calibrated" : "moved during calibration, press C again");
  imuN = 0;
  flush();
}

void sendImu() {
  const sensors::ImuSample& s = sens.imu().lastSample();
  const sensors::Vec3 w = sens.angularVelocity(), a = sens.imuAcceleration();
  msg::CalImuSample& c = imuBuf[imuN++];
  c.tUs = s.tUs;
  c.ax = s.acc.x;  c.ay = s.acc.y;  c.az = s.acc.z;
  c.gx = w.x;      c.gy = w.y;      c.gz = w.z;
  c.awx = a.x;     c.awy = a.y;     c.awz = a.z;
  c.yaw = sens.yaw();
  if (imuN < msg::CAL_IMU_PER_PACKET) return;
  m7.send(msg::PKT_CAL_IMU, imuBuf, sizeof(imuBuf));
  imuN = 0;
}

void sendTof() {
  const sensors::ToFReading& r = sens.tofReading();
  const sensors::Sensors::FusionDebug& f = sens.fusionDebug();
  msg::CalTof m;
  m.tUs = r.tUs;
  m.rangeM = r.rangeM;  m.height = sens.height();  m.signalMcps = r.signalMcps;
  m.valid = r.valid;  m.status = r.status;
  m.std = f.tofStd;
  m.result = (r.valid && f.tofUs == r.tUs) ? f.tofResult : -2;
  m.innov = sens.vertical().innovation();  m.innovStd = sens.vertical().innovationStd();
  m7.send(msg::PKT_CAL_TOF, &m, sizeof(m));
}

void sendCam(const msg::FlowHeader& fl) {
  const sensors::Sensors::FusionDebug& f = sens.fusionDebug();
  const sensors::Vec3 v = sens.cameraVelocity(), r = sens.cameraDebug().rot;
  const filters::HorizontalKF& h = sens.horizontal();
  msg::CalCam m;
  m.tUs = sens.cameraTimeUs();
  m.valid = sens.cameraValid();
  m.nInliers = fl.nInliers;
  m.vx = v.x;  m.vy = v.y;
  m.rmsPx = fl.rmsPx;  m.fx = fl.fx;  m.pairS = (fl.t1Us - fl.t0Us) * 1e-6f;
  m.rangeM = sens.tof().lastValid().rangeM;
  m.rot = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z);
  m.std = f.camStd;
  m.result = m.valid ? f.camResult : -2;
  m.innovX = h.innovation(0);  m.innovY = h.innovation(1);
  m.innovStdX = h.innovationStd(0);  m.innovStdY = h.innovationStd(1);
  m7.send(msg::PKT_CAL_CAM, &m, sizeof(m));
}

void got(uint8_t g) {
  if ((g & sensors::Sensors::NEW_IMU) && sens.imu().ready()) { sendImu(); nImu++; }
  if (g & sensors::Sensors::NEW_TOF) { sendTof(); nTof++; }
}

void setup() {
  comm::addLink(core);
  m7.begin("core");
  while (!sens.begin()) {
    say("IMU or ToF not answering, retrying");
    delay(1000);
  }
  say("sensors ok, press C to calibrate");
}

void loop() {
  static comm::Packet p;
  bool gotFlow = false;
  while (m7.receive(p)) {
    if (p.type == msg::PKT_FLOW && p.len >= sizeof(msg::FlowHeader)) {
      const msg::FlowHeader& f = *(const msg::FlowHeader*)p.data;
      if (p.len != sizeof(msg::FlowHeader) + f.count * sizeof(msg::PointMsg)) continue;
      got(sens.update(&f, (const msg::PointMsg*)(p.data + sizeof(msg::FlowHeader)), f.count));
      sendCam(f);
      nCam++;
      gotFlow = true;
    }
    else if (p.type == msg::PKT_FLIGHT_CMD && p.len == 1 && p.data[0] == msg::CMD_CALIBRATE) calibrate();
  }
  if (!gotFlow) got(sens.update());

  static uint32_t statUs = 0;
  if (shared::nowUs() - statUs >= 5000000) {
    statUs = shared::nowUs();
    say("imu %.0f/s  tof %.0f/s  camera %.0f/s", nImu / 5.0, nTof / 5.0, nCam / 5.0);
    nImu = nTof = nCam = 0;
  }
}
