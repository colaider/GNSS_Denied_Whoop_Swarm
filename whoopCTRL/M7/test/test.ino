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
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/cmeraReader.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/flowEstimator.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/GFTT.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/pyramidLK.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/RANSAC.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/closedCV/replenish.cpp"

// ---- objects (globals: they hold image buffers and pyramids) ----
comm::UsbLink         usb;
comm::Communication   comms;
vision::CameraReader  camera;
vision::FlowEstimator flow;
msg::PointMsg         pointMsgs[ccv::Replenish::MAX_POINTS];

void sendFlow(uint32_t flowUs) {
  const vision::FlowResult& r = flow.getResult();
  int n;
  const vision::PointFlow* pts = flow.getPoints(n);
  for (int i = 0; i < n; i++)
    pointMsgs[i] = {pts[i].id, pts[i].age, pts[i].x, pts[i].y, pts[i].dx, pts[i].dy, pts[i].inlier};

  msg::FlowHeader h = {r.seq, r.t0Us, r.t1Us, flowUs, r.tx, r.ty, r.angle, r.scale, r.rmsPx,
                       r.fx, r.fy, r.cx, r.cy, r.nTracked, r.nInliers, (uint16_t)n, (uint8_t)r.valid};
  comms.send(msg::PKT_FLOW, &h, sizeof(h), pointMsgs, (uint32_t)n * sizeof(msg::PointMsg));
}

// the shared clock (TIM5) must run at 1 MHz like micros(): both should show ~100000 us
void checkSharedClock() {
  const uint32_t c0 = shared::nowUs(), m0 = micros();
  delay(100);
  const uint32_t dc = shared::nowUs() - c0, dm = micros() - m0;
  char line[96];
  snprintf(line, sizeof(line), "shared clock check over delay(100): nowUs %lu us, micros %lu us",
           (unsigned long)dc, (unsigned long)dm);
  comms.log(line);
}

void setup() {
  comm::addLink(usb);
  comms.begin("usb");
  checkSharedClock();
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
    if (p.type == msg::PKT_CMD && p.len > 0 && p.data[0] == 'r') rawRequested = true;

  vision::Frame f;
  if (!camera.getFrame(f)) {
    comms.log("grab failed");
    return;
  }

  const uint32_t t0 = shared::nowUs();
  flow.update({f.data, f.width, f.height}, f.seq, f.tEndUs);
  const uint32_t flowUs = shared::nowUs() - t0;

  if (rawRequested) {                            // the untouched Bayer image of this same frame
    rawRequested = false;
    msg::RawHeader r = {f.seq, camera.rawWidth(), camera.rawHeight()};
    comms.send(msg::PKT_RAW, &r, sizeof(r), camera.rawData(), (uint32_t)r.width * r.height);
  }

  const vision::CameraStats& s = camera.stats();
  const calib::CameraIntrinsics& k = camera.cropIntrinsics();
  msg::FrameHeader h = {f.seq, f.tEndUs, s.lastGrabUs, s.lastProcessUs, s.grabErrors,
                        f.width, f.height, k.fx, k.fy, k.cx, k.cy};
  comms.send(msg::PKT_FRAME, &h, sizeof(h), f.data, (uint32_t)f.width * f.height);
  sendFlow(flowUs);
}
