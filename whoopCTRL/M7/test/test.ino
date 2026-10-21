// M7 camera test: sends camera frames to the PC, shown by frame_receiver.py (same folder).
// Close Serial Monitor first. Tools > Board: Arduino Nicla Vision, Target core: Main Core.
// The PC can send a PKT_CMD 'r' to get one raw Bayer image as well.

#ifndef CORE_CM7
#error "Set Tools > Target core: Main Core (M7)"
#endif

#include "camera.h"    // listed here so the Arduino IDE finds the Camera + GC2145 libraries
#include "gc2145.h"

// The IDE only compiles this sketch folder, so the project code is included by absolute path.
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/CAM/cmeraReader.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/communication.cpp"

// Debug message layouts (must match frame_receiver.py)
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

comm::Communication  comms;
vision::CameraReader camera;                     // global: it holds the image buffers

void setup() {
  comms.begin("usb");
  if (!camera.begin()) {
    while (true) {                               // keep telling the PC instead of failing silently
      comms.log("camera begin() failed: mode not calibrated, crop does not fit, or no RAM");
      delay(1000);
    }
  }
  comms.log("camera ok");
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
}
