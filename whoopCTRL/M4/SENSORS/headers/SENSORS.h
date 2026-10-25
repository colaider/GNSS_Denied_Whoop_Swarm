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
//     sens.pose();  sens.position();  sens.velocity();          // fused
//     sens.imuVelocity(); sens.cameraVelocity(); sens.gyroPosition(); ...   // each source alone
//
// Fusion (M4/FILTERS/KF.h): two Kalman filters, both predicted with every IMU sample:
//   horizontal: IMU acc x / y + camera velocity -> x, y, vx, vy (+ accelerometer bias)
//   vertical:   IMU acc z + ToF height          -> z, vz (+ bias)
// The camera (~70 ms late) and the ToF (~15 ms late) are applied at their own time.
//
// A small state machine per filter (Fix) decides what to do when its measurement is missing:
//   TRACKING  measurements arrive: the estimate is good
//   COASTING  none for a moment (camera blurred by a fast turn, ToF out of range): IMU alone,
//             it drifts (quadratically in position)
//   LOST      none for long: the velocity is pulled weakly to 0 (left alone it only runs away),
//             position / height just hold. The first measurement back is taken as it is.
// Still on the table (IMU: no rotation, exactly 1 g) without a measurement: velocity = 0.
// Each source also keeps its own estimate, for comparing. No heap memory, nothing to free.

#pragma once

#include <stdint.h>

#include "IMU.h"
#include "ToF.h"
#include "../visula/headers/visulaVel.h"
#include "../../FILTERS/headers/KF.h"
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

  // --- fusion ---
  filters::HorizontalConfig horizontal;
  filters::VerticalConfig   vertical;
  // How much a measurement is trusted (1 std). Still on the bench (sanity logs): camera
  // velocity 4..10 mm/s, ToF 1.4..1.5 mm. Camera: per point rmsPx * Z / (f dt), averaged over
  // the inliers, plus a floor and a share of the speed (height / derotation errors scale it).
  float camStdFloor = 0.008f;    // m/s
  float camStdScale = 0.05f;     // share of the speed
  float camMinPx    = 0.3f;      // smallest per-point error used, pixels
  float tofStd      = 0.002f;    // m: the sensor (1.5 mm) + floor texture (3 mm + 0.5%: ToF NIS 0.12)
  float tofStdRange = 0.002f;    // share of the range (grows with distance)
  float camStdRot   = 0.05f;     // + this share of the velocity the rotation alone would fake
                                 // (|rot| * Z / dt): derotation is never exact
  float tofMaxTiltDeg = 40.0f;   // ToF not used while tilted more (the beam hits walls, edges)

  // how much worse the IMU is while moving hard (attitude errors leak gravity, lever arms):
  // the filters' noise is accNoise * k, k = 1 + |gyro| / kRateRadS + | |acc| - 1 g | / kAccG
  float kRateRadS = 0.5f;
  float kAccG     = 0.1f;

  // --- state machine (see the top) ---
  float camCoastS  = 0.15f;      // no camera velocity this long: COASTING (3 frames)
  float camLostS   = 1.0f;       // ...this long: LOST
  float tofCoastS  = 0.1f;       // no ToF height this long: COASTING
  float tofLostS   = 0.5f;       // ...this long: LOST
  float lostVelStd = 0.3f;       // LOST: velocity = 0 +- this, 20 times a second
  // still: |gyro| < stillDps and |acc| within 1 g +- stillG, for stillS. Without a measurement
  // the velocity is then 0 +- stillVelStd. A flying drone is never this calm (vibration);
  // turn off (useStill) if that ever turns out wrong.
  bool  useStill    = true;
  float stillDps    = 3.0f;
  float stillG      = 0.03f;
  float stillS      = 0.3f;
  float stillVelStd = 0.02f;
};

// What a filter's measurement does right now (the state machine, see the top).
enum class Fix : uint8_t { TRACKING = 0, COASTING = 1, LOST = 2 };

struct Pose {                    // the fused output: world frame, m and rad
  float x = 0.0f, y = 0.0f, z = 0.0f, yaw = 0.0f;   // z = height above the floor
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

  // --- fused (world frame) ---
  Pose     pose() const;                                         // x, y, z, yaw
  Vec3     position() const;                                     // m
  Vec3     velocity() const;                                     // m/s
  float    yaw() const { return imu_.attitude().yaw; }           // rad (gyro only: drifts slowly)
  Vec3     positionStd() const;                                  // 1 std, m: how sure the filters are
  Vec3     velocityStd() const;                                  // 1 std, m/s
  Vec3     accBias() const;                                      // accelerometer bias found, m/s^2 (x / y body, z world)
  Fix      horizontalFix() const { return hFix_; }               // camera: TRACKING / COASTING / LOST
  Fix      verticalFix() const { return vFix_; }                 // ToF
  bool     still() const { return still_; }                      // the IMU says: not moving at all
  float    noiseScale() const { return k_; }                     // the k given to the filters now
  const filters::HorizontalKF& horizontal() const { return hkf_; }   // innovations, counters
  const filters::VerticalKF&   vertical() const { return vkf_; }

  // What the last camera / ToF update of the filters was, for checking the noise values: the
  // innovation (measured - predicted) should mostly stay within +-innovationStd.
  struct FusionDebug {
    float   camStd = 0.0f, tofStd = 0.0f;          // the std given to the filter
    int8_t  camResult = 0, tofResult = 0;          // the update's return: >0 applied, 0 refused,
                                                   // -1 too old, 3 reset (first one after LOST)
    uint32_t camUs = 0, tofUs = 0;                 // measurement times
  };
  const FusionDebug& fusionDebug() const { return fusDbg_; }

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
  void updateFix(uint32_t nowUs);                                 // the state machine + its made-up measurements
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

  filters::HorizontalKF hkf_;
  filters::VerticalKF   vkf_;
  FusionDebug           fusDbg_;
  Fix      hFix_ = Fix::TRACKING, vFix_ = Fix::TRACKING;
  uint32_t camSeenUs_ = 0, tofSeenUs_ = 0;    // last valid camera velocity (tMid) / ToF height
  uint32_t pseudoUs_  = 0;                    // last made-up measurement (LOST / still)
  bool     still_     = false;
  float    stillForS_ = 0.0f;                 // how long the IMU has looked still
  float    k_         = 1.0f;
};

}  // namespace sensors
