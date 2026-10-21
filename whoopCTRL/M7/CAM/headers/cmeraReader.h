#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "camera.h"
#include "gc2145.h"
#include "../../../calibartion/camera_calib.h"   // whoopCTRL/calibartion, made by gen_calib_headers.py

namespace vision {

constexpr int MAX_CROP_W = 160;          // largest output crop (320x240 image, 2x2 binning -> 160x120)
constexpr int MAX_CROP_H = 120;

struct CameraConfig {
  int32_t  sensorMode   = CAMERA_R320x240;  // what the chip reads out (sets scale + field of view)
  int32_t  windowMode   = CAMERA_R320x240;  // centre part transferred; = sensorMode -> whole image, no zoom
  uint16_t cropWidth    = 100;              // OUTPUT size; reads 2x this (200x200) and bins 2x2 -> grey
  uint16_t cropHeight   = 100;
  int16_t  cropShiftX   = 0;                // move the crop centre away from the image centre (camera pixels)
  int16_t  cropShiftY   = 0;
  uint16_t exposureRows = 0;                // 0 = auto exposure, else fixed exposure in sensor rows
  uint8_t  gain         = 0x40;             // global gain, used only with fixed exposure
};

struct Frame {
  const uint8_t* data = nullptr;   // grey, row-major, width*height bytes. Valid until the SECOND
                                   // getFrame() after this one (two alternating buffers), so
                                   // `prev = cur;` keeps the previous frame without copying pixels.
  uint16_t width  = 0;
  uint16_t height = 0;
  uint32_t seq    = 0;             // +1 per delivered frame
  uint32_t tEndUs = 0;             // when the readout finished (shared clock, microseconds)
};

struct CameraStats {
  uint32_t frames        = 0;      // frames delivered
  uint32_t grabErrors    = 0;      // failed / timed-out grabs
  uint32_t lastGrabUs    = 0;      // time spent waiting for the last frame
  uint32_t lastProcessUs = 0;      // time spent on crop + Bayer to grey
};

class CameraReader {
 public:
  explicit CameraReader(const CameraConfig& cfg = CameraConfig());

  // Sets up the sensor and allocates the raw buffer.
  // Returns false if the mode is not calibrated, the crop does not fit, or RAM runs out.
  bool begin();

  // Waits for the next frame (blocking, about one frame period) and fills `out`.
  // Returns false if the grab failed; `out` is then left unchanged.
  bool getFrame(Frame& out);

  const calib::CameraIntrinsics& cropIntrinsics() const { return cropK_; }  // for the output image
  const CameraStats&  stats()  const { return stats_; }
  const CameraConfig& config() const { return cfg_; }

  // Debug: the last raw Bayer image (whole transferred window), valid until the next getFrame().
  const uint8_t* rawData()   const { return raw_; }
  uint16_t       rawWidth()  const { return windowW_; }
  uint16_t       rawHeight() const { return windowH_; }

 private:
  // --- sensor ---
  bool    configureSensor();                       // cam_.begin(BAYER) + window + exposure
  void    setManualExposure(uint16_t rows, uint8_t gain);
  uint8_t regRead(uint8_t reg);                    // GC2145 register over CameraWire (0x3C)
  void    regWrite(uint8_t reg, uint8_t val);
  bool    grab();                                  // raw Bayer window into raw_, sets tEndUs_

  // --- processing ---
  bool    computeCropGeometry();                   // crop origin in the window + crop intrinsics
  void    extractCrop(uint8_t* dst);               // raw_ -> dst: 2x2 binning (R+G+G+B)/4 -> grey

  static const calib::CameraIntrinsics* intrinsicsForMode(int32_t mode);  // nullptr if not calibrated
  static bool modeSize(int32_t mode, uint16_t& w, uint16_t& h);

  CameraConfig cfg_;
  GC2145       sensor_;
  Camera       cam_{sensor_};
  FrameBuffer  fb_;

  uint8_t*  raw_     = nullptr;                    // raw Bayer window, 32-byte aligned, allocated in begin()
  uint16_t  windowW_ = 0, windowH_ = 0;            // size of the transferred window
  int16_t   cropX0_  = 0, cropY0_  = 0;            // raw crop origin in the window, kept even (each 2x2 = one Bayer cell)

  calib::CameraIntrinsics modeK_{};                // intrinsics of the full sensorMode image
  calib::CameraIntrinsics cropK_{};                // intrinsics of the output crop

  uint8_t   out_[2][MAX_CROP_W * MAX_CROP_H];      // two grey output buffers, used in turn
  uint8_t   outIdx_ = 0;                           // buffer the next frame goes into

  uint32_t    seq_    = 0;
  uint32_t    tEndUs_ = 0;
  CameraStats stats_;
};

}  // namespace vision
