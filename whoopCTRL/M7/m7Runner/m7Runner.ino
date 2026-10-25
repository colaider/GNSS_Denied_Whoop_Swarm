// M7 runner: camera + optical flow, the M4 link and the PC link.
//
// Every loop:
//   1. PC -> M4 : trajectory / flight-command packets are passed on unchanged (msg::forM4)
//   2. M4 -> PC : ACKs, telemetry, logs are passed on unchanged (msg::fromM4)
//   3. camera frame -> FlowEstimator
//   4. flow to the M4 with all points (RANSAC only marks inliers, the M4 decides what to use)
//   5. to the PC: the same flow packet (points if SEND_POINTS), the image every FRAME_EVERY frames
//
// Upload: Tools > Board: Arduino Nicla Vision, Target core: Main Core,
//         Flash split: "1MB M7 + 1MB M4" (the same split as the M4 sketch, or the M4 won't start).
// The M7 starts the M4 in setup(). Without an M4 program it waits 5 s, says so and goes on.
// View on the PC with M7/test/frame_receiver.py (close Serial Monitor first).

#ifndef CORE_CM7
#error "Set Tools > Target core: Main Core (M7)"
#endif

#include "camera.h"    // listed here so the Arduino IDE finds the Camera + GC2145 libraries
#include "gc2145.h"
#include <RPC.h>       // and the RPC library (used by CoreLink)

// The IDE only compiles this sketch folder, so the project code is included by absolute path.
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/cmeraReader.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/frameCapture.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/dcmiCapture.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/flowEstimator.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/GFTT.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/pyramidLK.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/RANSAC.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/replenish.cpp"

// ---- debug stream to the PC ----
const uint32_t FRAME_EVERY  = 3;       // send the image every N frames (0 = never). 10 KB over USB per
                                       // image: every frame made the loop miss the next one (21 -> 10 fps)
const bool     SEND_POINTS  = true;    // flow to the PC with its tracked points
const uint32_t STATS_EVERY_MS = 5000;  // a status line to the PC

// ---- objects (globals: they hold image buffers and pyramids) ----
comm::UsbLink         usb;
comm::CoreLink        core;
comm::Communication   pc;              // USB to the laptop (later: "wifi")
comm::Communication   m4;              // the M4 core
vision::CameraReader  camera;
vision::FlowEstimator flow;
msg::PointMsg         pointMsgs[ccv::Replenish::MAX_POINTS];
static_assert(ccv::Replenish::MAX_POINTS <= msg::FLOW_MAX_POINTS, "more points than a flow packet may carry");

bool     m4ok         = false;
bool     rawRequested = false;
uint32_t routedToM4 = 0, routedToPc = 0, frames = 0, flowUsSum = 0;
// where a frame's time goes (sums over the stats period, microseconds)
uint32_t grabSum = 0, pyramidSum = 0, trackSum = 0, ransacSum = 0, replenishSum = 0, sendSum = 0, searchSum = 0;

void logLine(const char* fmt, unsigned long a = 0, unsigned long b = 0, unsigned long c = 0, unsigned long d = 0,
             unsigned long e = 0, unsigned long g = 0) {
  char line[160];
  snprintf(line, sizeof(line), fmt, a, b, c, d, e, g);
  pc.log(line);
}

// ---- routing ----
void routeFromPc() {
  comm::Packet p;
  while (pc.receive(p)) {
    if (msg::forM4(p.type)) {
      if (m4ok && m4.send(p.type, p.data, p.len)) routedToM4++;
    } else if (p.type == msg::PKT_CMD && p.len > 0 && p.data[0] == 'r') {
      rawRequested = true;                     // debug: one raw Bayer image
    }
  }
}

void routeFromM4() {
  if (!m4ok) return;
  comm::Packet p;
  while (m4.receive(p))
    if (msg::fromM4(p.type) && pc.send(p.type, p.data, p.len)) routedToPc++;
}

// ---- flow output ----
void sendFlow(uint32_t flowUs) {
  const vision::FlowResult& r = flow.getResult();
  int n = 0;
  const vision::PointFlow* pts = flow.getPoints(n);
  for (int i = 0; i < n; i++) pointMsgs[i] = {pts[i].id, pts[i].age, pts[i].x, pts[i].y, pts[i].dx, pts[i].dy, pts[i].inlier};

  msg::FlowHeader h = {r.seq, r.t0Us, r.t1Us, flowUs, r.tx, r.ty, r.angle, r.scale, r.rmsPx, r.fx, r.fy, r.cx, r.cy, r.nTracked, r.nInliers, (uint16_t)n, (uint8_t)r.valid};
  const uint32_t pointBytes = (uint32_t)n * sizeof(msg::PointMsg);

  if (m4ok) m4.send(msg::PKT_FLOW, &h, sizeof(h), pointMsgs, pointBytes);   // all points, inliers marked

  if (!SEND_POINTS) h.count = 0;
  pc.send(msg::PKT_FLOW, &h, sizeof(h), pointMsgs, SEND_POINTS ? pointBytes : 0);
}

void sendFrame(const vision::Frame& f) {
  const vision::CameraStats& s = camera.stats();
  const calib::CameraIntrinsics& k = camera.cropIntrinsics();
  msg::FrameHeader h = {f.seq, f.tEndUs, s.lastGrabUs, s.lastProcessUs, s.grabErrors, f.width, f.height, k.fx, k.fy, k.cx, k.cy};
  pc.send(msg::PKT_FRAME, &h, sizeof(h), f.data, (uint32_t)f.width * f.height);
}

void sendStats() {
  static uint32_t last = shared::nowUs();
  const uint32_t now = shared::nowUs();
  if (now - last < STATS_EVERY_MS * 1000UL) return;
  const uint32_t fps10 = frames * 10000000UL / (now - last);      // frames per second x10
  logLine("M7: %lu.%lu fps, flow %lu us avg, routed to M4 %lu", fps10 / 10, fps10 % 10, frames ? flowUsSum / frames : 0, routedToM4);
  logLine("M7: routed from M4 %lu, core rx errors %lu, core overflow bytes %lu", routedToPc, m4.rxErrors(), core.rxOverflows());
  if (frames) {
    logLine("M7 per frame, avg us: grab wait %lu | pyramid %lu, track %lu, ransac %lu, replenish %lu | send %lu",
            grabSum / frames, pyramidSum / frames, trackSum / frames, ransacSum / frames, replenishSum / frames,
            sendSum / frames);
    logLine("M7: corner searches %lu.%02lu sectors per frame", searchSum / frames, searchSum * 100 / frames % 100);
    logLine("M7 camera: grab errors %lu, frames with a late time stamp %lu (both should stay 0)",
            camera.stats().grabErrors, camera.stats().inexactStamps);
    vision::SensorStatus ss;                   // ~3 ms of I2C: absorbed by the double-buffered capture
    if (camera.readSensor(ss)) {
      logLine("M7 camera: exposure %lu us (%lu rows, limit %lu), AEC %lu, gain global 0x%02lx pre 0x%02lx",
              ss.exposureUs, ss.exposureRows, camera.exposureLimitRows(), ss.aec, ss.globalGain, ss.preGain);
      logLine("M7 camera: post gain 0x%02lx, brightness %lu (AEC target %lu), image time = readout end - %lu us",
              ss.postGain, ss.brightness, ss.target, camera.exposureDelayUs());
    } else {
      logLine("M7 camera: sensor registers did not answer (I2C)");
    }
  }
  frames = flowUsSum = 0;
  grabSum = pyramidSum = trackSum = ransacSum = replenishSum = sendSum = searchSum = 0;
  last = now;
}

void setup() {
  comm::addLink(usb);
  comm::addLink(core);
  pc.begin("usb");
  pc.log("M7 runner starting");

  m4ok = m4.begin("core");                     // starts the M4, waits up to 5 s for it
  pc.log(m4ok ? "M4 started, core link ok" : "M4 not answering (no M4 program?) - running without it");

  if (!camera.begin()) {
    while (true) {                             // keep telling the PC instead of failing silently
      pc.log("camera begin() failed: mode not calibrated, crop does not fit, or no RAM");
      delay(1000);
    }
  }
  
  flow.begin(camera.cropIntrinsics());
  pc.log("camera + flow ok");
  const vision::CameraConfig& cc = camera.config();
  const unsigned long row100 = (unsigned long)(camera.rowTimeUs() * 100.0f + 0.5f);
  logLine("M7 camera: frame period %lu us, row %lu.%02lu us, exposure mode %lu (0 auto, 1 capped, 2 manual), limit %lu rows",
          camera.framePeriodUs(), row100 / 100, row100 % 100, (unsigned long)cc.exposure, camera.exposureLimitRows());
}

void loop() {
  routeFromPc();
  routeFromM4();

  vision::Frame f;
  if (!camera.getFrame(f)) {
    pc.log("grab failed");
    return;
  }

  const uint32_t t0 = shared::nowUs();
  flow.update({f.data, f.width, f.height}, f.seq, f.tUs);   // when the image was seen, for the M4's gyro
  const uint32_t flowUs = shared::nowUs() - t0;
  frames++;
  flowUsSum += flowUs;
  const vision::FlowTiming& ft = flow.getTiming();
  grabSum += camera.stats().lastGrabUs;
  pyramidSum += ft.pyramidUs;
  trackSum += ft.trackUs;
  ransacSum += ft.ransacUs;
  replenishSum += ft.replenishUs;
  searchSum += ft.searches;

  const uint32_t t1 = shared::nowUs();
  sendFlow(flowUs);                            // M4 first: it is the one that flies
  if (FRAME_EVERY && f.seq % FRAME_EVERY == 0) sendFrame(f);
  if (rawRequested) {                          // debug: the untouched Bayer image of this frame
    rawRequested = false;
    msg::RawHeader r = {f.seq, camera.rawWidth(), camera.rawHeight()};
    pc.send(msg::PKT_RAW, &r, sizeof(r), camera.rawData(), (uint32_t)r.width * r.height);
  }
  sendSum += shared::nowUs() - t1;

  routeFromM4();                               // again: the grab may have waited most of a frame
  sendStats();
}
