#include "headers/cmeraReader.h"

#include <stdlib.h>

#include "../../shared/headers/sharedClock.h"

namespace vision {

CameraReader::CameraReader(const CameraConfig& cfg) : cfg_(cfg) {}

// ---------------------------------------------------------------- public

bool CameraReader::begin() {
  if (!computeCropGeometry()) return false;

  if (!raw_) {
    // raw Bayer window, 1 byte per pixel. 32-byte aligned and a whole number of
    // 32-byte cache lines, because the library invalidates the D-cache over it after DMA.
    uint32_t bytes = ((uint32_t)windowW_ * windowH_ + 31) & ~31u;
    uint8_t* mem = (uint8_t*)malloc(bytes + 32);
    if (!mem) return false;
    raw_ = (uint8_t*)(((uintptr_t)mem + 31) & ~(uintptr_t)31);
    fb_.setBuffer(raw_);
  }
  return configureSensor();
}

bool CameraReader::getFrame(Frame& out) {
  if (!raw_ || !grab()) return false;

  uint8_t* dst = out_[outIdx_];
  outIdx_ ^= 1;                                // next frame goes into the other buffer

  uint32_t t0 = micros();
  extractCrop(dst);
  stats_.lastProcessUs = micros() - t0;

  out.data   = dst;
  out.width  = cfg_.cropWidth;
  out.height = cfg_.cropHeight;
  out.seq    = ++seq_;
  out.tEndUs = tEndUs_;
  stats_.frames++;
  return true;
}

// ---------------------------------------------------------------- sensor

bool CameraReader::configureSensor() {
  // GC2145 ignores the frame-rate argument; its timing is fixed by the mode.
  if (!cam_.begin(cfg_.sensorMode, CAMERA_BAYER, 30)) return false;

  // Transfer only the centre window, at the sensor mode's pixel scale.
  if (cfg_.windowMode != cfg_.sensorMode && cam_.zoomToCenter(cfg_.windowMode) != 0) return false;

  if (cfg_.exposureRows) setManualExposure(cfg_.exposureRows, cfg_.gain);
  return true;
}

void CameraReader::setManualExposure(uint16_t rows, uint8_t gain) {
  regWrite(0xFE, 0x00);                        // register page 0
  regWrite(0xB6, regRead(0xB6) & 0xFE);        // auto exposure off (bit 0)
  regWrite(0x03, (rows >> 8) & 0xFF);          // exposure time, in sensor rows
  regWrite(0x04, rows & 0xFF);
  regWrite(0xB0, gain);                        // global gain
}

uint8_t CameraReader::regRead(uint8_t reg) {
  CameraWire.beginTransmission((uint8_t)GC2145_I2C_ADDR);
  CameraWire.write(reg);
  CameraWire.endTransmission(false);           // repeated start, no stop
  CameraWire.requestFrom((uint8_t)GC2145_I2C_ADDR, (size_t)1);
  return CameraWire.available() ? CameraWire.read() : 0;
}

void CameraReader::regWrite(uint8_t reg, uint8_t val) {
  CameraWire.beginTransmission((uint8_t)GC2145_I2C_ADDR);
  CameraWire.write(reg);
  CameraWire.write(val);
  CameraWire.endTransmission();
}

bool CameraReader::grab() {
  uint32_t t0 = shared::nowUs();
  int err = cam_.grabFrame(fb_, 500);          // snapshot: waits for the next full frame
  tEndUs_ = shared::nowUs();                   // shared clock: the M4 can compare it directly
  stats_.lastGrabUs = tEndUs_ - t0;
  if (err != 0) { stats_.grabErrors++; return false; }
  return true;
}

// ---------------------------------------------------------------- processing

bool CameraReader::computeCropGeometry() {
  uint16_t sensorW, sensorH;
  if (!modeSize(cfg_.sensorMode, sensorW, sensorH)) return false;
  if (!modeSize(cfg_.windowMode, windowW_, windowH_)) return false;
  if (windowW_ > sensorW || windowH_ > sensorH) return false;

  const calib::CameraIntrinsics* k = intrinsicsForMode(cfg_.sensorMode);
  if (!k) return false;                        // this mode was never calibrated
  modeK_ = *k;

  const int cw = cfg_.cropWidth, ch = cfg_.cropHeight;
  if (cw < 2 || ch < 2 || cw > MAX_CROP_W || ch > MAX_CROP_H) return false;

  // window origin in sensor-mode pixels (same formula as Camera::zoomToCenter)
  const int winX0 = (sensorW - windowW_) / 2;
  const int winY0 = (sensorH - windowH_) / 2;

  // raw crop = 2x the output (2x2 binning), origin even so every 2x2 block is one Bayer cell
  const int rw = 2 * cw, rh = 2 * ch;
  int x0 = (windowW_ - rw) / 2 + cfg_.cropShiftX;
  int y0 = (windowH_ - rh) / 2 + cfg_.cropShiftY;
  x0 &= ~1; y0 &= ~1;
  if (x0 < 0 || y0 < 0 || x0 + rw > windowW_ || y0 + rh > windowH_) return false;
  cropX0_ = x0;
  cropY0_ = y0;

  // Binning halves the pixel scale. Output pixel j covers raw pixels 2j and 2j+1,
  // whose centre is raw 2j + 0.5  ->  j = (u_raw - 0.5) / 2.
  cropK_        = modeK_;
  cropK_.fx     = modeK_.fx * 0.5f;
  cropK_.fy     = modeK_.fy * 0.5f;
  cropK_.cx     = (modeK_.cx - (float)(winX0 + x0) - 0.5f) * 0.5f;
  cropK_.cy     = (modeK_.cy - (float)(winY0 + y0) - 0.5f) * 0.5f;
  cropK_.width  = cw;
  cropK_.height = ch;
  return true;
}

void CameraReader::extractCrop(uint8_t* dst) {
  const int cw = cfg_.cropWidth, ch = cfg_.cropHeight;
  for (int y = 0; y < ch; y++) {
    const uint8_t* r0 = raw_ + (uint32_t)(cropY0_ + 2 * y) * windowW_ + cropX0_;
    const uint8_t* r1 = r0 + windowW_;
    for (int x = 0; x < cw; x++) {
      const int i = 2 * x;
      *dst++ = (uint8_t)((r0[i] + r0[i + 1] + r1[i] + r1[i + 1] + 2) >> 2);
    }
  }
}

// ---------------------------------------------------------------- helpers

const calib::CameraIntrinsics* CameraReader::intrinsicsForMode(int32_t mode) {
  switch (mode) {
    case CAMERA_R160x120: return &calib::CAM_160x120;
    case CAMERA_R320x240: return &calib::CAM_320x240;
    case CAMERA_R640x480: return &calib::CAM_640x480;
    default:              return nullptr;
  }
}

bool CameraReader::modeSize(int32_t mode, uint16_t& w, uint16_t& h) {
  switch (mode) {
    case CAMERA_R160x120:   w = 160;  h = 120;  return true;
    case CAMERA_R320x240:   w = 320;  h = 240;  return true;
    case CAMERA_R320x320:   w = 320;  h = 320;  return true;
    case CAMERA_R640x480:   w = 640;  h = 480;  return true;
    case CAMERA_R800x600:   w = 800;  h = 600;  return true;
    case CAMERA_R1600x1200: w = 1600; h = 1200; return true;
    default:                return false;
  }
}

}  // namespace vision
