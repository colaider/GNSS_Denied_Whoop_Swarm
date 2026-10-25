// IMU - the LSM6DSOX on the M4: calibrated gyro + accelerometer, attitude, and the
// acceleration / velocity in the world frame.
//
// Reading the sensor (update) and the maths (step) are separate, so the maths can be tested
// on the PC with recorded samples.
//
// World frame, set by startup(): z up (against gravity), x / y level, yaw = 0 is the
// heading at startup. Only roll and pitch can be measured (from gravity); yaw comes from
// the gyro alone and drifts slowly (no magnetometer).
//
// The velocity is the integrated world acceleration (x, y only; z comes from the ToF). It
// drifts within seconds, so the estimator corrects it with the optical flow (setVelocity).
//
//   sensors::IMU imu;                     // global
//   imu.begin();                          // SPI + the calibrated sensor settings
//   imu.startup();                        // still for 2 s: gyro bias + level -> world frame
//   if (imu.update()) {                   // a new sample was read and processed
//     sensors::Attitude a = imu.attitude();
//     sensors::Vec3 v = imu.velocity();
//   }
//
// No heap memory, nothing to free.

#pragma once

#include <stdint.h>

#include "../../../calibartion/imu_calib.h"

namespace sensors {

struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct Attitude {                  // body relative to the world frame, radians
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

struct ImuSample {                 // one calibrated sample, body frame
  uint32_t tUs = 0;                // shared clock, when it was read
  Vec3     gyro;                   // rad/s about x, y, z (roll, pitch, yaw rate), bias NOT removed
  Vec3     acc;                    // g, specific force: gravity included
};

struct ImuConfig {
  float accelTrust = 1.0f;         // how fast the accelerometer pulls roll/pitch back (1/s)
  float accelGate  = 0.15f;        // use the accelerometer only if |acc| is within 1 g +- this,
  float accelGateDps = 60.0f;      // the rotation is slower than this,
  float accelCalmS = 0.1f;         // and both have been true for this long: while shaking, |acc|
                                   // passes 1 g for an instant at a tilted direction, and taking
                                   // it tilted the attitude by ~8 deg (gravity leak 1.4 m/s^2)
  float maxDtS     = 0.05f;        // larger gaps between samples are not integrated
};

class IMU {
 public:
  explicit IMU(const ImuConfig& cfg = ImuConfig());

  // SPI + sensor registers: 416 Hz, +-4 g, +-2000 dps, the settings the calibration is for.
  // False if the sensor does not answer (WHO_AM_I).
  bool begin();

  // Defines the world frame. The drone must be still for `ms`: measures a fresh gyro bias
  // (replaces the calibrated one) and roll / pitch from gravity; yaw = 0; velocity = 0.
  // False if it moved (then the calibrated bias stays and nothing changes).
  bool startup(uint32_t ms = 2000);

  // Reads a new sample if there is one and processes it with step(). True if it did.
  bool update();

  // The maths for one sample: attitude, world acceleration, velocity. Public for testing.
  void step(const ImuSample& s);

  // --- results of the last step ---
  Vec3     angularVelocity() const { return rate_; }    // rad/s, body frame, bias removed
  Vec3     acceleration() const { return last_.acc; }   // g, body frame, gravity included
  Attitude attitude() const;                             // rad
  const float* quaternion() const { return q_; }         // w, x, y, z: world from body
  Vec3     worldAcceleration() const { return accW_; }  // m/s^2, world frame, gravity removed
  Vec3     velocity() const { return vel_; }            // m/s, world frame, x / y only
  Vec3     gyroBias() const { return bias_; }           // rad/s
  const ImuSample& lastSample() const { return last_; }
  bool     ready() const { return started_; }            // startup() succeeded

  // The estimator replaces the drifting velocity with its corrected one.
  void setVelocity(float vx, float vy);

  // Rotates v from the body frame into the world frame for the attitude a.
  static Vec3 rotate(const Attitude& a, const Vec3& v);

 private:
  bool readSample(ImuSample& s);   // SPI burst read + calibration; false if no new sample
  void writeReg(uint8_t reg, uint8_t val);
  void readRegs(uint8_t reg, uint8_t* buf, int n);
  static Vec3 rotateQ(const float q[4], const Vec3& v);    // with the attitude quaternion

  ImuConfig cfg_;
  float     q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // attitude quaternion (w, x, y, z), world from body
  Vec3      bias_;                              // gyro bias, rad/s
  Vec3      rate_;                              // last angular velocity, bias removed
  Vec3      accW_;                              // last world acceleration, m/s^2
  Vec3      vel_;                               // world velocity, m/s
  ImuSample last_;
  bool      haveLast_ = false;                  // a previous sample exists (for dt)
  float     calmS_    = 0.0f;                   // how long the accelerometer has been usable
  bool      started_  = false;
};

}  // namespace sensors
