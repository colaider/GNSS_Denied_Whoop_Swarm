#include "headers/StateMachine.h"

#include <math.h>

#include "../../drones/drones.h"   // this drone: its calibration and settings, picked by the Nicla's ID
#include "../../shared/headers/sharedClock.h"

namespace ctrl {

// 'sm' prefix: sketches include several .cpp files into one compile unit
static float smClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static const float SM_DEG = 0.01745329f;           // rad per degree

StateMachine::StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg) : sens_(sens), fc_(fc), pos_(pos), cfg_(cfg) {}

bool StateMachine::begin() {
  fc_.begin();
  return sens_.begin();
}

const char* StateMachine::stateName() const {
  static const char* const names[] = {"BOOT", "READY", "SPINUP", "CLIMB", "FLY", "?", "LAND"};
  return names[state_];
}

bool StateMachine::calibrate() {
  heard();
  if (state_ != READY) return false;
  refOk_ = false;                                  // a new offset after every calibration
  calibrated_ = sens_.startup();                   // blocks 2 s
  now_ = shared::nowUs();                          // fresh time for the FC class after the block
  fc_.update(now_);
  takeReference();                                 // the FC link often looks lost right after the block:
  heard();                                         // then step() takes it as soon as the FC answers again
  return calibrated_;                              // (log 23.11 21:58: never taken, roll / pitch ran on the
}                                                  // gyro alone to -18 / +25 deg, x / y moved on z changes)

void StateMachine::takeReference() {               // still on the ground: Betaflight's roll / pitch vs ours
  if (refOk_ || !calibrated_ || state_ != READY || !fc_.linkOk()) return;
  const drone::FcAttitude a = fc_.attitude();
  const sensors::Attitude n = sens_.attitude();
  refRollOff_  = n.roll  - a.rollDeg  * SM_DEG;
  refPitchOff_ = n.pitch - a.pitchDeg * SM_DEG;
  refOk_ = true;
}

bool StateMachine::start() {
  heard();
  if (state_ != READY || !calibrated_ || !refOk_) return false;   // no flight without the roll / pitch reference
  if (!drones::known()) return false;               // this Nicla is not in drones/drones.h: raw IMU, no camera
  const drones::Drone& d = drones::active();
  pos_.setGains(d.kp, d.kd, d.ki);                 // this drone's (drones.h); the PC may change them in flight
  calibrated_ = false; go(SPINUP);
  return true;
}
void StateMachine::land() { heard(); if (state_ == SPINUP || state_ == CLIMB || state_ == FLY) go(LAND); }
void StateMachine::kill() { heard(); fc_.disarm(); if (state_ != BOOT) go(READY); }
void StateMachine::setTarget(const sensors::Pose& target, const sensors::Pose& vel, const sensors::Pose& acc) { target_ = target; tVel_ = vel; tAcc_ = acc; }

sensors::Pose StateMachine::limited(sensors::Pose t) const {   // the only place the limits are applied
  t.x = smClamp(t.x, origin_.x - cfg_.maxDistM, origin_.x + cfg_.maxDistM);
  t.y = smClamp(t.y, origin_.y - cfg_.maxDistM, origin_.y + cfg_.maxDistM);
  t.z = smClamp(t.z, 0.0f, cfg_.maxHeightM);
  return t;
}

// entry actions only
void StateMachine::go(State s) {
  const sensors::Pose p = sens_.pose();
  switch (s) {
    case READY:
      fc_.disarm();
      cmd_ = PosCTRL::Command{};
      break;

    case SPINUP:
      fc_.arm();
      thr_ = 0.0f;
      armedUs_ = now_;
      break;

    case CLIMB: {                                  // the controller takes over where SPINUP left it: no jump
      pos_.reset();
      sp_ = p;
      climbFrom_ = p.z;
      climbTo_ = limited(target_).z;
      climbS_ = fmaxf(1.875f * fabsf(climbTo_ - climbFrom_) / cfg_.climbRate, 0.5f);   // min-jerk peak speed 1.875 d / T
      break;
    }

    case FLY:
      origin_ = sp_;
      break;

    case LAND:
      sp_.x = p.x;  sp_.y = p.y;  sp_.z = p.z;
      break;

    default: break;
  }
  spVel_ = spAcc_ = sensors::Pose();
  if (s == READY || s == SPINUP) xyOn_ = false;
  pos_.climbGains(s == CLIMB);                     // one controller: the climb z gains in CLIMB, the hover ones after
  state_ = s; enterUs_ = now_; enterZ_ = p.z; heardUs_ = now_;
}


void StateMachine::step(uint32_t nowUs) {
  dt_ = now_ ? (int32_t)(nowUs - now_) * 1e-6f : 0.0f;
  now_ = nowUs; fc_.update(nowUs);
  const float v = fc_.batteryV();
  const float hoverV = drones::active().hoverThrottleV;   // throttle x volts (drones.h); a fixed 0.34 was 8 % too
  if (hoverV > 0.0f && v > 5.0f) {                 // much on a full battery. Hover throttle from the battery (0 V: not read yet)
    vBat_ = vBat_ > 0.0f ? vBat_ + (v - vBat_) * fminf(dt_ / cfg_.batteryTauS, 1.0f) : v;
    pos_.config().throttleHover = hoverV / vBat_;
  }
  const float hover = pos_.config().throttleHover;

  switch (state_) {
    case BOOT:
      if (fc_.linkReady() && fc_.setupModes(true)) go(READY);
      break;

    case READY:
      takeReference();                             // if calibrate() could not (FC link lost then)
      break;

    case SPINUP: {                                 // throttle 0 -> hover, level, no feedback
      if (!fc_.armed()) { armedUs_ = now_; thr_ = 0.0f; break; }
      const float k = (uint32_t)(now_ - armedUs_) * 1e-6f / cfg_.spinupS;
      thr_ = fminf(k, 1.0f) * hover;
      if (k >= 1.0f) go(CLIMB);
      break;
    }

    case CLIMB: {                                  // setpoint z: min-jerk from climbFrom_ to climbTo_ in climbS_
      const float T = climbS_, d = climbTo_ - climbFrom_;
      const float s = (uint32_t)(now_ - enterUs_) * 1e-6f;
      const float tau = fminf(s / T, 1.0f);
      const float t2 = tau * tau, t3 = t2 * tau;
      sp_.z    = climbFrom_ + d * t3 * (10.0f - 15.0f * tau + 6.0f * t2);
      spVel_.z = d * 30.0f * t2 * (1.0f - tau) * (1.0f - tau) / T;
      const float tl = fminf((s + cfg_.climbLeadS) / T, 1.0f);   // the acceleration climbLeadS ahead (0 past the end)
      spAcc_.z = d * 60.0f * tl * (1.0f - tl) * (1.0f - 2.0f * tl) / (T * T);
      if (tau >= 1.0f) go(FLY);
      break;
    }

    case FLY: {                                    // setpoint towards the target, at most maxSpeed
      const sensors::Pose t = limited(target_);
      const sensors::Pose d = t - sp_;
      const float dist = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z), step = cfg_.maxSpeed * dt_;
      if (dist <= step) {
        sp_ = t;  spVel_ = tVel_;  spAcc_ = tAcc_;
      } else {                                     // the target jumped: follow it at maxSpeed
        sp_ += d * (step / dist);
        spVel_ = d * (cfg_.maxSpeed / dist);
        spAcc_ = sensors::Pose();
        sp_.yaw = t.yaw;  spVel_.yaw = 0.0f;
      }
      break;
    }

    case LAND:
      sp_.z -= cfg_.descendRate * dt_;
      spVel_.z = -cfg_.descendRate;
      if ((uint32_t)(now_ - enterUs_) > (uint32_t)((fmaxf(enterZ_, 0.0f) / cfg_.descendRate + cfg_.landExtraS) * 1e6f)) go(READY);
      break;
  }

  const bool lost = cfg_.linkTimeoutMs && (uint32_t)(now_ - heardUs_) > cfg_.linkTimeoutMs * 1000UL;
  if (flying() && !fc_.armed() && !fc_.arming()) go(READY);
  if (lost && (state_ == SPINUP || state_ == CLIMB || state_ == FLY)) go(LAND);
  sens_.flying(flying());                          // height from the ToF while the motors shake the accelerometer
  sens_.grounded(state_ == READY || state_ == SPINUP);   // still on the floor: z = 0, vz = 0 (the ToF reads 0 mm)

  if (refOk_ && fc_.linkOk()) {                    // roll / pitch drift corrected by Betaflight's attitude
    const drone::FcAttitude a = fc_.attitude();
    sens_.attitudeReference(a.rollDeg * SM_DEG + refRollOff_, a.pitchDeg * SM_DEG + refPitchOff_, now_);
  }

  if (flying()) control();
}


void StateMachine::control() {
  if (sens_.horizontalFix() != sensors::Fix::TRACKING) camBadUs_ = now_;
  if (state_ == SPINUP) {
    if (!fc_.armed()) return;
    cmd_ = PosCTRL::Command{};
    cmd_.throttle = thr_;
    fc_.setAttitude(0.0f, 0.0f, 0.0f, thr_);
    return;
  }

  const sensors::Pose p = sens_.pose();
  if (!xyOn_ && (uint32_t)(now_ - camBadUs_) >= (uint32_t)(cfg_.xyAfterS * 1e6f)) {
    xyOn_ = true;
    origin_.x += p.x - sp_.x;  origin_.y += p.y - sp_.y;
    sp_.x = p.x;  sp_.y = p.y;
  }
  if (sens_.horizontalFix() == sensors::Fix::LOST) xyOn_ = false;
  if (state_ == LAND && sp_.z < cfg_.levelBelowM) xyOn_ = false;

  cmd_ = pos_.update(p, sens_.poseVelocity(), sp_, dt_, spVel_, spAcc_);
  if (!xyOn_) { cmd_.rollDeg = cmd_.pitchDeg = 0.0f; pos_.resetXY(); }
  fc_.setAttitude(cmd_.rollDeg, cmd_.pitchDeg, cmd_.yawRateDps, cmd_.throttle);
}

}  // namespace ctrl
