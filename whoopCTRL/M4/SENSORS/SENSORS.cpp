#include "headers/SENSORS.h"

#include <math.h>

#include "../../shared/headers/sharedClock.h"

namespace sensors {

// A Vec3 component by axis number (0 = x, 1 = y, 2 = z). 'sens' prefix: sketches include
// several .cpp files into one compile unit, where file-private names must not collide.
static float sensGet(const Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
static void  sensSet(Vec3& v, int i, float f) { if (i == 0) v.x = f; else if (i == 1) v.y = f; else v.z = f; }

Sensors::Sensors(const SensorsConfig& cfg)
    : cfg_(cfg), imu_(cfg.imu), tof_(cfg.tof), hkf_(cfg.horizontal), vkf_(cfg.vertical) {}

bool Sensors::begin() {
  const bool imuOk = imu_.begin();
  const bool tofOk = tof_.begin();                 // tried even if the IMU failed
  return imuOk && tofOk;
}

bool Sensors::startup(uint32_t ms) {
  if (!imu_.startup(ms)) return false;
  imuPos_ = camVel_ = camPos_ = rotSum_ = Vec3();
  camValid_ = false;
  camUs_    = 0;
  histN_ = histHead_ = 0;
  tofN_ = tofHead_ = 0;
  imuPosUs_ = imu_.lastSample().tUs;               // integration starts from the last startup sample
  addGyro(imu_.lastSample());                      // ...and so does the gyro history (rotation 0)
  const uint32_t now = shared::nowUs();            // not lastSample(): startup() does not update it
  hkf_.reset(now);                                 // the filters: at rest at (0, 0, height)
  vkf_.reset(now, height());
  hkf_.setMaxDt(cfg_.imu.maxDtS);
  vkf_.setMaxDt(cfg_.imu.maxDtS);
  fusDbg_ = FusionDebug();
  hFix_ = vFix_ = Fix::TRACKING;                   // still at startup: as good as measured
  camSeenUs_ = tofSeenUs_ = pseudoUs_ = now;
  still_ = false;
  stillForS_ = 0.0f;
  k_ = 1.0f;
  return true;
}

uint8_t Sensors::update(const msg::FlowHeader* flow, const msg::PointMsg* pts, int n) {
  uint8_t got = 0;

  if (imu_.update()) {
    got |= NEW_IMU;
    if (imu_.ready()) {
      const ImuSample& s = imu_.lastSample();
      const float dt = (s.tUs - imuPosUs_) * 1e-6f;
      if (dt > 0.0f && dt <= cfg_.imu.maxDtS) {    // a gap is not integrated (like the IMU)
        imuPos_.x += imu_.velocity().x * dt;
        imuPos_.y += imu_.velocity().y * dt;
      }
      imuPosUs_ = s.tUs;
      addGyro(s);

      // how hard it moves: rotation rate and how far |acc| is from 1 g
      const Vec3 w = imu_.angularVelocity();
      const float rate = sqrtf(w.x * w.x + w.y * w.y + w.z * w.z);
      const float dev  = fabsf(sqrtf(s.acc.x * s.acc.x + s.acc.y * s.acc.y + s.acc.z * s.acc.z) - 1.0f);
      k_ = 1.0f + rate / cfg_.kRateRadS + dev / cfg_.kAccG;
      const bool stillNow = rate < cfg_.stillDps * 0.017453293f && dev < cfg_.stillG;
      stillForS_ = stillNow ? stillForS_ + (dt > 0.0f && dt <= cfg_.imu.maxDtS ? dt : 0.0f) : 0.0f;
      still_ = cfg_.useStill && stillForS_ >= cfg_.stillS;

      const Vec3 a = imu_.worldAcceleration();     // both filters step with every sample
      hkf_.predict(s.tUs, a.x, a.y, imu_.attitude().yaw, k_);
      vkf_.predict(s.tUs, a.z, k_);
    }
  }

  if (tof_.update()) {
    got |= NEW_TOF;
    if (tof_.last().valid) {                       // into the range history (for the trend)
      tofT_[tofHead_] = tof_.last().tUs;
      tofR_[tofHead_] = tof_.last().rangeM;
      tofHead_ = (tofHead_ + 1) % TOF_HIST;
      if (tofN_ < TOF_HIST) tofN_++;

      // into the vertical filter, unless tilted too much (h / range = cos of the beam's tilt)
      const float h = height();
      const float range = tof_.last().rangeM;
      if (imu_.ready() && h > range * cosf(cfg_.tofMaxTiltDeg * 0.017453293f)) {
        const uint32_t t = tof_.last().tUs;
        fusDbg_.tofStd = cfg_.tofStd + cfg_.tofStdRange * range;
        if (vFix_ == Fix::LOST) {                  // back after long: take it as it is
          fusDbg_.tofResult = vkf_.setHeight(t, h, fusDbg_.tofStd) ? 3 : -1;
        } else {
          fusDbg_.tofResult = (int8_t)vkf_.updateHeight(t, h, fusDbg_.tofStd);
        }
        fusDbg_.tofUs = t;
        tofSeenUs_ = t;
      }
    }
  }

  if (flow) {
    got |= NEW_CAMERA;
    const bool hadCam = camValid_;
    camValid_ = false;

    // the ToF reading must be from about the time of the frames (signed: it may be newer)
    const ToFReading& tof = tof_.lastValid();
    const int32_t tofAge = (int32_t)(flow->t1Us - tof.tUs);
    const bool tofOk = tof.valid && tofAge < 200000 && tofAge > -200000;

    // how much closer the floor got between the two frames: the ToF trend over the frame time
    float rate = 0.0f;
    const float dZ = rangeRate(rate) ? rate * (flow->t1Us - flow->t0Us) * 1e-6f : 0.0f;

    // the gyro's rotation over the two frames (kept for checking even when there is no velocity)
    Vec3 rotBody;
    const bool haveRot = imu_.ready() && rotationBetween(flow->t0Us, flow->t1Us, rotBody);
    camDbg_ = CameraDebug();
    camDbg_.dZ = dZ;
    if (haveRot) camDbg_.rot = toCamera(rotBody);

    if (haveRot && tofOk && flow->valid && flow->nInliers > 0 && n > 0) {
      const visualVel::visualVelocityVector v =
          vis_.estimateVelocity(flow->t1Us, imu_.attitude(), camDbg_.rot, tof, dZ, *flow, pts, n);
      camDbg_.vx = v.vx;
      camDbg_.vy = v.vy;

      Vec3 cam;                                    // camera axes; z (along the view) not measured yet
      cam.x = v.vx;
      cam.y = v.vy;
      const uint32_t tMid = flow->t0Us + (flow->t1Us - flow->t0Us) / 2;

      // into the world with the attitude at tMid, not now (the packet is ~40 ms old): undo the
      // body rotation since tMid, v - since x v (small angle)
      Vec3 body = toBody(cam), since;
      if (rotationBetween(tMid, imu_.lastSample().tUs, since)) {
        const Vec3 b = body;
        body.x = b.x - (since.y * b.z - since.z * b.y);
        body.y = b.y - (since.z * b.x - since.x * b.z);
        body.z = b.z - (since.x * b.y - since.y * b.x);
      }
      const Vec3 world = IMU::rotate(imu_.attitude(), body);

      const float dt = (tMid - camUs_) * 1e-6f;
      if (hadCam && dt > 0.0f && dt < 0.2f) {      // after a gap the position just stops
        camPos_.x += world.x * dt;
        camPos_.y += world.y * dt;
      }
      camVel_.x = world.x;
      camVel_.y = world.y;
      camVel_.z = 0.0f;
      camPos_.z = height();
      camUs_    = tMid;
      camValid_ = true;

      // into the horizontal filter at tMid. Per point: the tracking error (rmsPx) turned into
      // m/s like the velocity (* Z / (f dt)); averaged over the inliers it shrinks by sqrt(n)
      // While rotating, the rotation's own image motion (|rot| * Z / dt as a speed) is far
      // larger than the translation and is never removed exactly: a share of it is added.
      const float px = flow->rmsPx > cfg_.camMinPx ? flow->rmsPx : cfg_.camMinPx;
      const float pairS = (flow->t1Us - flow->t0Us) * 1e-6f;
      const float perPoint = px * tof.rangeM / (flow->fx * pairS);
      const float speed = sqrtf(world.x * world.x + world.y * world.y) * cfg_.camStdScale;
      const Vec3& r = camDbg_.rot;
      const float rotV = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z) * tof.rangeM / pairS * cfg_.camStdRot;
      fusDbg_.camStd = sqrtf(perPoint * perPoint / flow->nInliers + cfg_.camStdFloor * cfg_.camStdFloor +
                             speed * speed + rotV * rotV);
      if (hFix_ == Fix::LOST) {                    // back after long: the filter's velocity is a guess, take this one
        fusDbg_.camResult = hkf_.setVelocity(tMid, world.x, world.y, fusDbg_.camStd) ? 3 : -1;
      } else {
        fusDbg_.camResult = (int8_t)hkf_.updateVelocity(tMid, world.x, world.y, fusDbg_.camStd);
      }
      fusDbg_.camUs = tMid;
      camSeenUs_ = tMid;
    }
  }
  if (imu_.ready()) updateFix(imu_.lastSample().tUs);
  return got;
}

// The state machine: what each filter does without its measurement (see SENSORS.h).
void Sensors::updateFix(uint32_t nowUs) {
  auto fix = [](uint32_t age, float coastS, float lostS) {
    return age > lostS * 1e6f ? Fix::LOST : (age > coastS * 1e6f ? Fix::COASTING : Fix::TRACKING);
  };
  const int32_t camAge = (int32_t)(nowUs - camSeenUs_), tofAge = (int32_t)(nowUs - tofSeenUs_);
  hFix_ = fix(camAge > 0 ? camAge : 0, cfg_.camCoastS, cfg_.camLostS);
  vFix_ = fix(tofAge > 0 ? tofAge : 0, cfg_.tofCoastS, cfg_.tofLostS);

  // made-up measurements, 20 times a second: still -> v = 0 (good), LOST -> v = 0 (weak)
  if ((uint32_t)(nowUs - pseudoUs_) < 50000) return;
  pseudoUs_ = nowUs;
  if (hFix_ != Fix::TRACKING && (still_ || hFix_ == Fix::LOST))
    hkf_.zeroVelocity(nowUs, still_ ? cfg_.stillVelStd : cfg_.lostVelStd);
  if (vFix_ != Fix::TRACKING && (still_ || vFix_ == Fix::LOST))
    vkf_.zeroVelocity(nowUs, still_ ? cfg_.stillVelStd : cfg_.lostVelStd);
}

Pose Sensors::pose() const {
  Pose p;
  p.x   = hkf_.estimate().x[filters::HorizontalKF::PX];
  p.y   = hkf_.estimate().x[filters::HorizontalKF::PY];
  p.z   = vkf_.estimate().x[filters::VerticalKF::Z];
  p.yaw = yaw();
  return p;
}

Vec3 Sensors::position() const {
  const Pose p = pose();
  Vec3 v;
  v.x = p.x;
  v.y = p.y;
  v.z = p.z;
  return v;
}

Vec3 Sensors::velocity() const {
  Vec3 v;
  v.x = hkf_.estimate().x[filters::HorizontalKF::VX];
  v.y = hkf_.estimate().x[filters::HorizontalKF::VY];
  v.z = vkf_.estimate().x[filters::VerticalKF::VZ];
  return v;
}

// 1 std of a state: the square root of its variance (the diagonal of P)
Vec3 Sensors::positionStd() const {
  using H = filters::HorizontalKF;
  using V = filters::VerticalKF;
  Vec3 v;
  v.x = sqrtf(hkf_.estimate().P[H::PX][H::PX]);
  v.y = sqrtf(hkf_.estimate().P[H::PY][H::PY]);
  v.z = sqrtf(vkf_.estimate().P[V::Z][V::Z]);
  return v;
}

Vec3 Sensors::velocityStd() const {
  using H = filters::HorizontalKF;
  using V = filters::VerticalKF;
  Vec3 v;
  v.x = sqrtf(hkf_.estimate().P[H::VX][H::VX]);
  v.y = sqrtf(hkf_.estimate().P[H::VY][H::VY]);
  v.z = sqrtf(vkf_.estimate().P[V::VZ][V::VZ]);
  return v;
}

Vec3 Sensors::accBias() const {
  Vec3 v;
  v.x = hkf_.estimate().x[filters::HorizontalKF::BX];
  v.y = hkf_.estimate().x[filters::HorizontalKF::BY];
  v.z = vkf_.estimate().x[filters::VerticalKF::BZ];
  return v;
}

float Sensors::height() const {
  const ToFReading& r = tof_.lastValid();
  if (!r.valid) return 0.0f;
  Vec3 view;                                       // the ToF beam: along the camera's view (side by side on the board)
  view.z = 1.0f;
  const Vec3 beam = toBody(view);
  return ToF::height(r.rangeM, imu_.quaternion(), beam.x, beam.y, beam.z);
}

void Sensors::addGyro(const ImuSample& s) {
  if (histN_ > 0) {
    const float dt = (s.tUs - histT_[(histHead_ + HIST - 1) % HIST]) * 1e-6f;
    if (dt > cfg_.imu.maxDtS) {
      histN_ = 0;                                  // a gap: the rotation during it is unknown
    } else {
      const Vec3 w = imu_.angularVelocity();       // bias removed
      rotSum_.x += w.x * dt;
      rotSum_.y += w.y * dt;
      rotSum_.z += w.z * dt;
    }
  }
  histT_[histHead_]   = s.tUs;
  histRot_[histHead_] = rotSum_;
  histHead_ = (histHead_ + 1) % HIST;
  if (histN_ < HIST) histN_++;
}

bool Sensors::rangeRate(float& out) {
  if (tofN_ < 3) return false;
  // time relative to the newest reading, in seconds: small numbers, so linReg's float sums stay exact
  const uint32_t newest = tofT_[(tofHead_ + TOF_HIST - 1) % TOF_HIST];
  float t[TOF_HIST], r[TOF_HIST];
  int n = 0;
  for (int i = 0; i < tofN_; i++) {
    const uint32_t age = newest - tofT_[i];
    if (age > 300000) continue;                    // only the last 0.3 s (older ones are from before a gap)
    t[n] = -(float)age * 1e-6f;
    r[n] = tofR_[i];
    n++;
  }
  if (n < 3) return false;
  out = vis_.linReg(t, r, n).v;                    // slope = range change per second
  return true;
}

bool Sensors::rotationBetween(uint32_t t0Us, uint32_t t1Us, Vec3& out) const {
  // the summed rotation at time t: interpolated between the two samples around it
  auto at = [this](uint32_t t, Vec3& r) -> bool {
    const int newest = (histHead_ + HIST - 1) % HIST;
    for (int k = 0; k < histN_ - 1; k++) {         // newest pairs first: frames are recent
      const int b = (newest - k + HIST) % HIST;
      const int a = (b + HIST - 1) % HIST;         // the sample before b
      const uint32_t span = histT_[b] - histT_[a];
      const uint32_t into = t - histT_[a];         // wraps to huge if t is before a
      if (into <= span) {
        const float f = span ? (float)into / span : 0.0f;
        r.x = histRot_[a].x + (histRot_[b].x - histRot_[a].x) * f;
        r.y = histRot_[a].y + (histRot_[b].y - histRot_[a].y) * f;
        r.z = histRot_[a].z + (histRot_[b].z - histRot_[a].z) * f;
        return true;
      }
    }
    return false;                                  // older than the history, or newer than the last sample
  };

  Vec3 r0, r1;
  if (!at(t0Us, r0) || !at(t1Us, r1)) return false;
  out.x = r1.x - r0.x;
  out.y = r1.y - r0.y;
  out.z = r1.z - r0.z;
  return true;
}

Vec3 Sensors::toCamera(const Vec3& body) const {
  Vec3 c;
  for (int i = 0; i < 3; i++) sensSet(c, i, cfg_.camSign[i] * sensGet(body, cfg_.camAxis[i]));
  return c;
}

Vec3 Sensors::toBody(const Vec3& cam) const {
  Vec3 b;
  for (int i = 0; i < 3; i++) sensSet(b, cfg_.camAxis[i], cfg_.camSign[i] * sensGet(cam, i));
  return b;
}

}  // namespace sensors
