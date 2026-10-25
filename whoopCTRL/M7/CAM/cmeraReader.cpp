#include "headers/cmeraReader.h"

#include <stdlib.h>

#include "../../shared/headers/sharedClock.h"

namespace vision {

CameraReader::CameraReader(const CameraConfig& cfg) : cfg_(cfg) {}

// ---------------------------------------------------------------- public

bool CameraReader::begin() {
  if (!computeCropGeometry()) return false;

  if (!rawBuf_[0]) {
    // two raw Bayer windows, 1 byte per pixel. 32-byte aligned and a whole number of
    // 32-byte cache lines: the D-cache is invalidated over them around each DMA.
    rawBytes_ = ((uint32_t)windowW_ * windowH_ + 31) & ~31u;
    for (int i = 0; i < 2; i++) {
      uint8_t* mem = (uint8_t*)malloc(rawBytes_ + 32);
      if (!mem) return false;
      rawBuf_[i] = (uint8_t*)(((uintptr_t)mem + 31) & ~(uintptr_t)31);
    }
    fb_.setBuffer(rawBuf_[0]);
  }
  if (!configureSensor()) return false;

  // exposure with the nominal row time, then the real row time from a few library grabs
  // (they also check the camera answers and leave the DCMI + DMA set up for FrameCapture),
  // then the exposure again if the row time was off
  rowUs_ = cfg_.nominalRowUs;
  if (!applyExposure() || !measureRowTime()) return false;
  if (rowUs_ < cfg_.nominalRowUs * 0.97f || rowUs_ > cfg_.nominalRowUs * 1.03f) applyExposure();

  // the crop's centre row -> the last row of the transferred window, in sensor rows
  centreToEndRows_ = (uint16_t)((windowH_ - (cropY0_ + cfg_.cropHeight)) * subsampleForMode(cfg_.sensorMode));
  SensorStatus s;
  readSensor(s);                               // sets the exposure delay from the real exposure
  if (cfg_.exposureDelayUs) delayUs_ = cfg_.exposureDelayUs;

  cap_.begin(rawBuf_[0], rawBuf_[1], rawBytes_);   // from now on: frames into the two buffers in turn
  return true;
}

bool CameraReader::readSensor(SensorStatus& s) {
  bool ok = regWrite(0xFE, 0x00);
  s.exposureRows = (uint16_t)(((regRead(0x03) & 0x1F) << 8) | regRead(0x04));
  s.globalGain   = regRead(0xB0);
  s.preGain      = regRead(0xB1);
  s.postGain     = regRead(0xB2);
  s.aec          = regRead(0xB6) & 0x01;
  ok &= regWrite(0xFE, 0x01);
  s.target       = regRead(0x13);
  s.brightness   = regRead(0x14);
  ok &= regWrite(0xFE, 0x00);
  s.exposureUs   = (uint32_t)(s.exposureRows * rowUs_ + 0.5f);
  if (ok && !cfg_.exposureDelayUs) delayUs_ = exposure::delayUs(centreToEndRows_, s.exposureRows, rowUs_);
  return ok;
}

bool CameraReader::getFrame(Frame& out) {
  if (!rawBuf_[0] || !grab()) return false;

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
  out.tUs    = tEndUs_ - delayUs_;
  stats_.frames++;
  return true;
}

// ---------------------------------------------------------------- sensor

bool CameraReader::configureSensor() {
  // GC2145 ignores the frame-rate argument; its timing is fixed by the mode.
  if (!cam_.begin(cfg_.sensorMode, CAMERA_BAYER, 30)) return false;

  // Transfer only the centre window, at the sensor mode's pixel scale.
  if (cfg_.windowMode != cfg_.sensorMode && cam_.zoomToCenter(cfg_.windowMode) != 0) return false;

  return true;
}

bool CameraReader::applyExposure() {
  exposure::Reg r[24];
  int n = 0;
  switch (cfg_.exposure) {
    case ExposureMode::AUTO:
      limitRows_ = 0;
      return true;                             // the library's settings as they are
    case ExposureMode::AUTO_CAPPED:
      limitRows_ = exposure::usToRows(cfg_.maxExposureUs, rowUs_);
      n = exposure::capRegs(limitRows_, cfg_.maxPostGain, 0, r);
      break;
    case ExposureMode::MANUAL:
      limitRows_ = exposure::usToRows(cfg_.manualExposureUs, rowUs_);
      n = exposure::manualRegs(limitRows_, cfg_.gain, r);
      break;
  }
  return writeRegs(r, n);
}

bool CameraReader::writeRegs(const exposure::Reg* r, int n) {
  bool ok = true;
  for (int i = 0; i < n; i++) ok &= regWrite(r[i].reg, r[i].value);   // the page is set by 0xfe writes in the list
  return ok;
}

bool CameraReader::measureRowTime() {
  // Back-to-back snapshot grabs catch consecutive frames (nothing done in between), so the
  // time between their ends is the frame period. The first ones let the AEC settle under the
  // new limit (a longer exposure would stretch the frame); the shortest later gap is taken.
  uint32_t last = 0, best = 0xFFFFFFFFu;
  for (int i = 0; i < 8; i++) {
    if (cam_.grabFrame(fb_, 500) != 0) return false;
    const uint32_t t = shared::nowUs();
    if (i >= 4 && t - last < best) best = t - last;
    last = t;
  }
  regWrite(0xFE, 0x00);
  const uint16_t vb   = (uint16_t)(((regRead(0x07) & 0x1F) << 8) | regRead(0x08));
  const uint16_t winH = (uint16_t)(((regRead(0x0D) & 0x07) << 8) | regRead(0x0E));
  if (vb + winH == 0) return false;
  periodUs_ = best;
  rowUs_    = exposure::rowTimeUs(best, vb, winH);
  return true;
}

uint8_t CameraReader::regRead(uint8_t reg) {
  CameraWire.beginTransmission((uint8_t)GC2145_I2C_ADDR);
  CameraWire.write(reg);
  CameraWire.endTransmission(false);           // repeated start, no stop
  CameraWire.requestFrom((uint8_t)GC2145_I2C_ADDR, (size_t)1);
  return CameraWire.available() ? CameraWire.read() : 0;
}

bool CameraReader::regWrite(uint8_t reg, uint8_t val) {
  CameraWire.beginTransmission((uint8_t)GC2145_I2C_ADDR);
  CameraWire.write(reg);
  CameraWire.write(val);
  return CameraWire.endTransmission() == 0;
}

bool CameraReader::grab() {
  bool exact = true;
  const uint8_t* raw = cap_.next(500000, tEndUs_, exact);   // tEndUs_: shared clock, the M4 compares it directly
  stats_.lastGrabUs = cap_.lastWaitUs();
  if (!raw) { stats_.grabErrors++; return false; }
  if (!exact) stats_.inexactStamps++;
  raw_ = (uint8_t*)raw;
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

int CameraReader::subsampleForMode(int32_t mode) {
  switch (mode) {                              // GC2145::setResolutionWithZoom: window = output x this
    case CAMERA_R160x120: return 4;
    case CAMERA_R320x240:
    case CAMERA_R320x320: return 3;
    case CAMERA_R640x480: return 2;
    default:              return 1;
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
