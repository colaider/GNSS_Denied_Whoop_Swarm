#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "camera.h"
#include "gc2145.h"
#include "frameCapture.h"
#include "exposure.h"
#include "../../../calibartion/camera_calib.h"   // whoopCTRL/calibartion, made by gen_calib_headers.py

namespace vision {

constexpr int MAX_CROP_W = 160;          // largest output crop (320x240 image, 2x2 binning -> 160x120)
constexpr int MAX_CROP_H = 120;

enum class ExposureMode : uint8_t {
  AUTO,         // the sensor's AEC alone: ~36-43 ms here indoors (nearly a whole frame) -> blur
  AUTO_CAPPED,  // AEC, but never longer than maxExposureUs; darker scenes get more gain instead
  MANUAL        // fixed exposure + fixed gain, AEC off
};

struct CameraConfig {
  int32_t  sensorMode   = CAMERA_R320x240;  // what the chip reads out (sets scale + field of view)
  int32_t  windowMode   = CAMERA_R320x240;  // centre part transferred; = sensorMode -> whole image, no zoom
  uint16_t cropWidth    = 100;              // OUTPUT size; reads 2x this (200x200) and bins 2x2 -> grey
  uint16_t cropHeight   = 100;
  int16_t  cropShiftX   = 0;                // move the crop centre away from the image centre (camera pixels)
  int16_t  cropShiftY   = 0;
  // Motion blur = how far the image moves during the exposure. With the AEC alone (~40 ms)
  // 50 deg/s smears 2.8 px of the 100 px image, and tracking collapsed above ~25 deg/s
  // (sanity logs 24.10: 4 inliers at 25-50 deg/s, 0.3 at 50-100). Capped at 10 ms the blur
  // is 4x shorter; the AEC raises the gain instead (more noise: needs a lit room).
  // 10 ms = one flicker period of 50 Hz mains lighting, so the brightness does not pulse from
  // frame to frame (LK assumes it constant). In daylight / DC LEDs it could be shorter.
  ExposureMode exposure     = ExposureMode::AUTO_CAPPED;
  uint32_t maxExposureUs    = 10000;        // AUTO_CAPPED: the longest exposure the AEC may use
  uint8_t  maxPostGain      = 0xc0;         // AUTO_CAPPED: most digital gain it may use (P1:0x20;
                                            // datasheet default; the library had limited it to 0x40)
  uint32_t manualExposureUs = 5000;         // MANUAL: fixed exposure
  uint8_t  gain             = 0x40;         // MANUAL: global gain (4.4 fixed point, 0x40 = 4.0)
  float    nominalRowUs     = 59.8f;        // sensor row time until it is measured in begin()
  // How long before the end of the readout the image was seen: the centre row's readout to the
  // last row's (rolling shutter) + half the exposure. 0: computed (exposure::delayUs) from the
  // measured row time and the exposure the sensor reports (readSensor() updates it). Else this
  // fixed value. Check: the AEC alone was measured at 37-42 ms (gyro vs image motion); the
  // formula gives 39.5-43 ms for its 600-720 rows.
  uint32_t exposureDelayUs  = 0;
};

struct SensorStatus {              // what the sensor reports (CameraReader::readSensor)
  uint16_t exposureRows = 0;       // P0:0x03/04
  uint32_t exposureUs   = 0;       // the same with the measured row time
  uint8_t  globalGain = 0, preGain = 0, postGain = 0;   // P0:0xb0 / 0xb1 / 0xb2
  uint8_t  brightness = 0;         // P1:0x14, current frame average
  uint8_t  target     = 0;         // P1:0x13, what the AEC aims at
  bool     aec        = false;     // P0:0xb6 bit 0
};

struct Frame {
  const uint8_t* data = nullptr;   // grey, row-major, width*height bytes. Valid until the SECOND
                                   // getFrame() after this one (two alternating buffers), so
                                   // `prev = cur;` keeps the previous frame without copying pixels.
  uint16_t width  = 0;
  uint16_t height = 0;
  uint32_t seq    = 0;             // +1 per delivered frame
  uint32_t tEndUs = 0;             // when the readout finished (shared clock, microseconds)
  uint32_t tUs    = 0;             // when the image was seen: tEndUs - exposureDelayUs. Use this one
                                   // to match the image with other sensors (gyro, ToF).
};

struct CameraStats {
  uint32_t frames        = 0;      // frames delivered
  uint32_t grabErrors    = 0;      // failed / timed-out grabs
  uint32_t lastGrabUs    = 0;      // time spent waiting for the last frame (it is read in while
                                   // the previous one is processed, so: frame period - processing)
  uint32_t inexactStamps = 0;      // frames whose end time is only when it was noticed (late)
  uint32_t lastProcessUs = 0;      // time spent on crop + Bayer to grey
};

class CameraReader {
 public:
  explicit CameraReader(const CameraConfig& cfg = CameraConfig());

  // Sets up the sensor (window, exposure mode), measures the row time, allocates the two raw
  // buffers and starts reading frames in. Takes ~0.3 s (a few frames).
  // Returns false if the mode is not calibrated, the crop does not fit, or RAM runs out.
  bool begin();

  // Waits for the frame being read in (blocking, up to one frame period minus the time since
  // the last call), starts the next one, and fills `out`.
  // Returns false if the grab failed; `out` is then left unchanged.
  bool getFrame(Frame& out);

  // Reads the exposure / gain / brightness registers (I2C, ~3 ms: call rarely, e.g. with the
  // stats) and, with exposureDelayUs = 0, updates the delay from the exposure. False on I2C error.
  bool readSensor(SensorStatus& s);

  float    rowTimeUs() const { return rowUs_; }              // measured in begin()
  uint32_t framePeriodUs() const { return periodUs_; }       // measured in begin()
  uint32_t exposureDelayUs() const { return delayUs_; }      // what Frame.tUs subtracts now
  uint16_t exposureLimitRows() const { return limitRows_; }  // cap (AUTO_CAPPED) / fixed (MANUAL), 0: none

  const calib::CameraIntrinsics& cropIntrinsics() const { return cropK_; }  // for the output image
  const CameraStats&  stats()  const { return stats_; }
  const CameraConfig& config() const { return cfg_; }

  // Debug: the last raw Bayer image (whole transferred window), valid until the next getFrame().
  // (The next frame is being written into the other raw buffer meanwhile.)
  const uint8_t* rawData()   const { return raw_; }
  uint16_t       rawWidth()  const { return windowW_; }
  uint16_t       rawHeight() const { return windowH_; }

 private:
  // --- sensor ---
  bool    configureSensor();                       // cam_.begin(BAYER) + window + exposure
  bool    applyExposure();                         // the exposure mode's registers, with rowUs_
  bool    measureRowTime();                        // a few library grabs: frame period -> rowUs_, periodUs_
  bool    writeRegs(const exposure::Reg* r, int n);
  uint8_t regRead(uint8_t reg);                    // GC2145 register over CameraWire (0x3C)
  bool    regWrite(uint8_t reg, uint8_t val);     // false: no I2C acknowledge
  bool    grab();                                  // next raw Bayer window -> raw_, sets tEndUs_

  // --- processing ---
  bool    computeCropGeometry();                   // crop origin in the window + crop intrinsics
  void    extractCrop(uint8_t* dst);               // raw_ -> dst: 2x2 binning (R+G+G+B)/4 -> grey

  static const calib::CameraIntrinsics* intrinsicsForMode(int32_t mode);  // nullptr if not calibrated
  static bool modeSize(int32_t mode, uint16_t& w, uint16_t& h);
  static int  subsampleForMode(int32_t mode);      // sensor rows per output row (the library's setting)

  CameraConfig cfg_;
  GC2145       sensor_;
  Camera       cam_{sensor_};
  FrameBuffer  fb_;

  uint8_t*  rawBuf_[2] = {nullptr, nullptr};       // two raw Bayer windows, 32-byte aligned, allocated in begin()
  uint32_t  rawBytes_  = 0;                        // size of each (whole cache lines)
  uint8_t*  raw_     = nullptr;                    // the one holding the last delivered frame
  FrameCapture cap_;                               // reads frames into rawBuf_ in turn
  uint16_t  windowW_ = 0, windowH_ = 0;            // size of the transferred window
  int16_t   cropX0_  = 0, cropY0_  = 0;            // raw crop origin in the window, kept even (each 2x2 = one Bayer cell)

  calib::CameraIntrinsics modeK_{};                // intrinsics of the full sensorMode image
  calib::CameraIntrinsics cropK_{};                // intrinsics of the output crop

  uint8_t   out_[2][MAX_CROP_W * MAX_CROP_H];      // two grey output buffers, used in turn
  uint8_t   outIdx_ = 0;                           // buffer the next frame goes into

  float     rowUs_     = 0.0f;                     // sensor row time, us
  uint32_t  periodUs_  = 0;                        // frame period, us
  uint16_t  limitRows_ = 0;
  uint16_t  centreToEndRows_ = 0;                  // sensor rows from the crop's centre row to the last one read
  uint32_t  delayUs_   = 40000;

  uint32_t    seq_    = 0;
  uint32_t    tEndUs_ = 0;
  CameraStats stats_;
};

}  // namespace vision
