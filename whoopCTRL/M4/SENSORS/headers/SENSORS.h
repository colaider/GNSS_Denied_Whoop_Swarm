// Sensors - every sensor of the M4 in one place: IMU, ToF and the camera's visual velocity.
//
// update() runs every loop and never waits: it reads the IMU and the ToF when they have new
// data, and turns a flow packet (if one arrived from the M7) into a camera velocity. The IMU
// is propagated into the visual estimation here: every IMU sample goes into a short gyro
// history, and each flow packet gets the camera's rotation over exactly its two frames
// (t0 -> t1) from it, for derotation.
//
//   sensors::Sensors sens;                     // global
//   sens.begin();
//   sens.startup();                            // drone still: gyro bias + world frame
//   loop:
//     if (flow packet arrived) sens.update(&flow, pts, n);  else sens.update();
//     sens.imuVelocity(); sens.cameraVelocity(); sens.attitude(); ...
//
// Each source keeps its own estimate (for comparing them); combining them is the job of the
// filter in M4/FILTERS. No heap memory, nothing to free.

#pragma once

#include <stdint.h>

#include "IMU.h"
#include "ToF.h"
#include "../visula/headers/visulaVel.h"
#include "../../../shared/headers/messages.h"

namespace sensors {

struct SensorsConfig {
  ImuConfig imu;
  ToFConfig tof;
  // How the camera sits on the body: camera axis i = camSign[i] * body axis camAxis[i]
  // (camera: x right and y down in the image, z along the view; body: the IMU's x, y, z).
  // Measured on the board (sanity log, 233 rotating frames): camera x = body y, camera y =
  // body x, camera z = -body z (looking down). The image is turned 90 deg against the IMU's
  // x / y; with it the gyro's predicted image motion matches the measured one (r 0.82 / 0.95,
  // before: r ~0). It must stay a rotation (right-handed: camera x cross camera y = camera z),
  // never a mirror, or the gyro's rotation turns the wrong way in camera axes.
  // The ToF beam is taken along camera z.
  uint8_t camAxis[3] = {1, 0, 2};
  int8_t  camSign[3] = {1, 1, -1};
};

class Sensors {
 public:
  enum : uint8_t { NEW_IMU = 1, NEW_TOF = 2, NEW_CAMERA = 4 };   // what update() got

  explicit Sensors(const SensorsConfig& cfg = SensorsConfig());

  // IMU + ToF. False if one does not answer.
  bool begin();

  // The drone must be still: gyro bias and the world frame (IMU::startup), positions = 0.
  bool startup(uint32_t ms = 2000);

  // Every loop. flow = the newest flow packet from the M7, or nullptr when none arrived.
  // Returns NEW_* flags of what was new this time.
  uint8_t update(const msg::FlowHeader* flow = nullptr, const msg::PointMsg* pts = nullptr, int n = 0);

  // --- IMU (world frame) ---
  Vec3     imuPosition() const { return imuPos_; }               // m, x / y: drifts within seconds alone
  Vec3     imuVelocity() const { return imu_.velocity(); }       // m/s, x / y
  Vec3     imuAcceleration() const { return imu_.worldAcceleration(); }   // m/s^2, gravity removed

  // --- gyro ---
  Vec3     gyroVelocity() const { return imu_.angularVelocity(); }  // rad/s, body frame, bias removed
  Attitude gyroPosition() const { return imu_.attitude(); }         // roll / pitch / yaw, rad

  // --- camera (world frame) ---
  Vec3     cameraVelocity() const { return camVel_; }            // m/s, x / y (z: not measured yet)
  Vec3     cameraPosition() const { return camPos_; }            // m, x / y integrated, z = ToF height
  uint32_t cameraTimeUs() const { return camUs_; }               // when it was measured (middle of the frame pair)
  bool     cameraValid() const { return camValid_; }             // the last flow packet gave a velocity

  // What the last flow packet's camera velocity was made of, for checking it step by step.
  struct CameraDebug {
    Vec3  rot;                                   // gyro rotation over the frame pair, camera axes, rad
    float dZ = 0.0f;                             // distance change over the frame pair (ToF trend), m
    float vx = 0.0f, vy = 0.0f;                  // estimateVelocity's result, camera axes, m/s
  };
  const CameraDebug& cameraDebug() const { return camDbg_; }

  // --- ToF ---
  float    height() const;                                       // m above the floor, tilt corrected (last valid)
  const ToFReading& tofReading() const { return tof_.last(); }

  // the sensors themselves, for anything not above
  const IMU& imu() const { return imu_; }
  const ToF& tof() const { return tof_; }

 private:
  void addGyro(const ImuSample& s);                               // into the gyro history
  bool rotationBetween(uint32_t t0Us, uint32_t t1Us, Vec3& out) const;   // body, rad; false: not in the history
  bool rangeRate(float& out);                                     // ToF trend, m/s (linReg); false: too few readings
  Vec3 toCamera(const Vec3& body) const;                          // body axes -> camera axes
  Vec3 toBody(const Vec3& cam) const;                             // camera axes -> body axes

  SensorsConfig cfg_;
  IMU           imu_;
  ToF           tof_;
  visualVel     vis_;

  // gyro history: the rotation summed since startup (body, rad) at every IMU sample, so the
  // rotation between two camera frames is the difference of two (interpolated) entries
  static constexpr int HIST = 128;            // 128 samples at 416 Hz = 307 ms: a frame pair + its delay
  uint32_t histT_[HIST];
  Vec3     histRot_[HIST];
  int      histHead_ = 0, histN_ = 0;
  Vec3     rotSum_;

  // the last valid ToF readings, for the range trend (how fast the floor gets closer)
  static constexpr int TOF_HIST = 12;         // ~270 ms at 44 Hz
  uint32_t tofT_[TOF_HIST];
  float    tofR_[TOF_HIST];
  int      tofHead_ = 0, tofN_ = 0;

  Vec3     imuPos_;                           // integrated imu velocity
  uint32_t imuPosUs_  = 0;
  Vec3     camVel_, camPos_;
  uint32_t camUs_     = 0;
  bool     camValid_  = false;
  CameraDebug camDbg_;
};

}  // namespace sensors
