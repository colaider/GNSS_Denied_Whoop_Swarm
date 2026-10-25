// The two fusion filters of the M4. The IMU's world acceleration drives both (predict at every
// sample, 416 Hz: smooth, no delay); the slower, late but drift-free sensors correct them.
//
//   HorizontalKF   x = [px, py, vx, vy, bx, by]   IMU acc x / y predicts, camera velocity corrects
//                  (bx, by: accelerometer bias in BODY x / y, turned into the world with the yaw,
//                   so it stays put when the drone turns)
//   VerticalKF     x = [z, vz, bz]                IMU acc z predicts, ToF height corrects
//
// Position x / y is never measured (the camera gives velocity): it is the integral of the
// corrected velocity and drifts slowly, but stays in the filter for absolute fixes later.
// Late measurements (camera ~70 ms, ToF ~15 ms) are applied at their own time and the IMU
// steps after them replayed (DelayedKF in kalman.h).
//
//   filters::HorizontalKF h;  filters::VerticalKF v;
//   h.reset(t); v.reset(t, height);
//   every IMU sample:  h.predict(t, accW.x, accW.y, yaw, k);  v.predict(t, accW.z, k);
//                      (k >= 1: how much worse than usual the IMU is right now, e.g. shaking)
//   camera velocity:   h.updateVelocity(tMid, vx, vy, std);
//   ToF height:        v.updateHeight(tTof, height, std);

#pragma once

#include <stdint.h>

#include "kalman.h"

namespace filters {

struct HorizontalConfig {
  // How wrong the IMU acceleration is while calm, m/s^2/sqrt(Hz); predict()'s k scales it
  // up while rotating / shaking. The sensor itself is ~0.006 (8 mg at 416 Hz, bench); what
  // counts is the attitude error (1 deg tilt = 0.17 m/s^2) and, in flight, vibration.
  // Larger: follows the camera (and its noise) more; smaller: smoother but lags.
  // 0.3 gave camera NIS 0.04 on the bench (sanity 21:18): far too large, the fused velocity
  // was as noisy as the camera's. 0.08 ~ NIS 1.
  float accNoise  = 0.08f;
  float biasWalk  = 0.002f;      // how fast the bias may change, m/s^2/sqrt(s)
  float biasInit  = 0.2f;        // bias uncertainty at reset, m/s^2 (~20 mg)
  float gate      = 5.0f;        // camera velocity more than this many std off: refused
  int   forceAfter = 5;          // ...this many refused in a row: taken as the new velocity
};

struct VerticalConfig {
  float accNoise  = 0.05f;       // as above, z; a tilt error barely changes z (cos). 0.1: ToF NIS 0.12
  float biasWalk  = 0.002f;
  float biasInit  = 0.2f;
  float gate      = 5.0f;        // ToF height more than this many std off: refused
  int   forceAfter = 3;          // ...this many in a row: a step in the floor, height jumps to it
};

class HorizontalKF : public DelayedKF<6, 4, 96> {   // u = [ax, ay, yaw, k]; 96 steps = 230 ms back
 public:
  enum { PX, PY, VX, VY, BX, BY };
  explicit HorizontalKF(const HorizontalConfig& cfg = HorizontalConfig()) : cfg_(cfg) {}

  void reset(uint32_t tUs);                                   // at rest at (0, 0), bias unknown
  // world acc (gravity removed), m/s^2; yaw, rad; k: noise scale (accNoise * k)
  void predict(uint32_t tUs, float ax, float ay, float yaw, float k = 1.0f);

  // World velocity measured at tUs, std per axis (m/s). Both axes gated together. Returns 2
  // applied, 0 refused as an outlier, -1 too old. Last innovations: innovation().
  int updateVelocity(uint32_t tUs, float vx, float vy, float std);

  // The velocity IS (vx, vy) +- std at tUs, whatever the filter thought (the camera is back
  // after being lost). False if tUs is too old.
  bool setVelocity(uint32_t tUs, float vx, float vy, float std);

  // A weak "probably not moving" measurement, v = 0 +- std (no camera for long / IMU still).
  void zeroVelocity(uint32_t tUs, float std);

  float innovation(int axis) const { return innov_[axis]; }  // measured - predicted, last update
  float innovationStd(int axis) const;                        // what it should be, 1 std
  uint32_t forced() const { return forced_; }                 // times the camera was forced in

 protected:
  void model(float dt, const float u[4], float F[6][6], float b[6], float Q[6][6]) const override;

 private:
  HorizontalConfig cfg_;
  float    innov_[2] = {}, innovVar_[2] = {};
  uint32_t forced_ = 0;
};

class VerticalKF : public DelayedKF<3, 2, 48> {     // u = [az, k]; 48 steps = 115 ms back
 public:
  enum { Z, VZ, BZ };
  explicit VerticalKF(const VerticalConfig& cfg = VerticalConfig()) : cfg_(cfg) {}

  void reset(uint32_t tUs, float z);                          // at rest at height z, bias unknown
  void predict(uint32_t tUs, float az, float k = 1.0f);       // world acc z (gravity removed), m/s^2; k as above

  // Height above the floor measured at tUs, std (m). 1 applied, 0 refused, -1 too old;
  // 2: a floor step (refused forceAfter times in a row), the height jumped to it.
  int updateHeight(uint32_t tUs, float z, float std);

  void zeroVelocity(uint32_t tUs, float std);                 // vz = 0 +- std (no ToF for long / still)
  bool setHeight(uint32_t tUs, float z, float std);           // the height IS z (ToF back after LOST)

  float innovation() const { return innov_; }
  float innovationStd() const;
  uint32_t steps() const { return steps_; }                   // floor steps so far

 protected:
  void model(float dt, const float u[2], float F[3][3], float b[3], float Q[3][3]) const override;

 private:
  VerticalConfig cfg_;
  float    innov_ = 0.0f, innovVar_ = 0.0f;
  uint32_t steps_ = 0;
};

}  // namespace filters
