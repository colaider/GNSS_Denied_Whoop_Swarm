// M7 camera + optical flow test. Every frame: grab -> FlowEstimator -> send the image
// (PKT_FRAME) and the flow result with its tracked points (PKT_FLOW) to the PC.
// Shown live by frame_receiver.py (same folder). Close Serial Monitor first.
// Tools > Board: Arduino Nicla Vision, Target core: Main Core.
// The PC can send a PKT_CMD 'r' to get one raw Bayer image as well.

#ifndef CORE_CM7
#error "Set Tools > Target core: Main Core (M7)"
#endif

#include "camera.h"    // listed here so the Arduino IDE finds the Camera + GC2145 libraries
#include "gc2145.h"

// The IDE only compiles this sketch folder, so the project code is included by absolute path.
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/cmeraReader.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/flowEstimator.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/GFTT.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/pyramidLK.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/RANSAC.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/replenish.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/communication.cpp"

// ---- message layouts (must match frame_receiver.py) ----
struct __attribute__((packed)) FrameHeader {     // PKT_FRAME payload: this + pixels
  uint32_t seq, tEndUs, grabUs, procUs, grabErrors;
  uint16_t width, height;
  float    fx, fy, cx, cy;                       // intrinsics of the output image
};
static_assert(sizeof(FrameHeader) == 40, "must match the Python struct '<5I2H4f'");

struct __attribute__((packed)) RawHeader {       // PKT_RAW payload: this + raw Bayer bytes
  uint32_t seq;
  uint16_t width, height;
};
static_assert(sizeof(RawHeader) == 8, "must match the Python struct '<I2H'");

struct __attribute__((packed)) FlowHeader {      // PKT_FLOW payload: this + count * PointMsg
  uint32_t seq, t0Us, t1Us, flowUs;              // flowUs: time FlowEstimator::update took
  float    tx, ty, angle, scale, rmsPx, fx, fy;  // frame motion (pixels, radians)
  uint16_t nTracked, nInliers, count;            // count = PointMsg entries that follow
  uint8_t  valid;
};
static_assert(sizeof(FlowHeader) == 51, "must match the Python struct '<4I7f3HB'");

struct __attribute__((packed)) PointMsg {        // one tracked point
  uint16_t id, age;
  float    x, y, dx, dy;                         // position now, motion since the last frame
  uint8_t  inlier;
};
static_assert(sizeof(PointMsg) == 21, "must match the Python struct '<2H4fB'");

// ---- objects (globals: they hold image buffers and pyramids) ----
comm::Communication   comms;
vision::CameraReader  camera;
vision::FlowEstimator flow;
PointMsg              pointMsgs[ccv::Replenish::MAX_POINTS];

void sendFlow(uint32_t flowUs) {
  const vision::FlowResult& r = flow.getResult();
  int n;
  const vision::PointFlow* pts = flow.getPoints(n);
  for (int i = 0; i < n; i++)
    pointMsgs[i] = {pts[i].id, pts[i].age, pts[i].x, pts[i].y, pts[i].dx, pts[i].dy, pts[i].inlier};

  FlowHeader h = {r.seq, r.t0Us, r.t1Us, flowUs, r.tx, r.ty, r.angle, r.scale, r.rmsPx, r.fx, r.fy,
                  r.nTracked, r.nInliers, (uint16_t)n, (uint8_t)r.valid};
  comms.send(comm::PKT_FLOW, &h, sizeof(h), pointMsgs, (uint32_t)n * sizeof(PointMsg));
}

void setup() {
  comms.begin("usb");
  if (!camera.begin()) {
    while (true) {                               // keep telling the PC instead of failing silently
      comms.log("camera begin() failed: mode not calibrated, crop does not fit, or no RAM");
      delay(1000);
    }
  }
  flow.begin(camera.cropIntrinsics());
  comms.log("camera + flow ok");
}

void loop() {
  static bool rawRequested = false;
  comm::Packet p;
  while (comms.receive(p))
    if (p.type == comm::PKT_CMD && p.len > 0 && p.data[0] == 'r') rawRequested = true;

  vision::Frame f;
  if (!camera.getFrame(f)) {
    comms.log("grab failed");
    return;
  }

  const uint32_t t0 = micros();
  flow.update({f.data, f.width, f.height}, f.seq, f.tEndUs);
  const uint32_t flowUs = micros() - t0;

  if (rawRequested) {                            // the untouched Bayer image of this same frame
    rawRequested = false;
    RawHeader r = {f.seq, camera.rawWidth(), camera.rawHeight()};
    comms.send(comm::PKT_RAW, &r, sizeof(r), camera.rawData(), (uint32_t)r.width * r.height);
  }

  const vision::CameraStats& s = camera.stats();
  const calib::CameraIntrinsics& k = camera.cropIntrinsics();
  FrameHeader h = {f.seq, f.tEndUs, s.lastGrabUs, s.lastProcessUs, s.grabErrors,
                   f.width, f.height, k.fx, k.fy, k.cx, k.cy};
  comms.send(comm::PKT_FRAME, &h, sizeof(h), f.data, (uint32_t)f.width * f.height);
  sendFlow(flowUs);
}
