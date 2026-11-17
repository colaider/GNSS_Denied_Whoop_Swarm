#include "headers/StateMachine.h"

#include <math.h>

#include "../../shared/headers/sharedClock.h"

namespace ctrl {

// 'sm' prefix: sketches include several .cpp files into one compile unit
static float smClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static const float SM_DEG = 0.01745329f;           // rad per degree

StateMachine::StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg) : sens_(sens), fc_(fc), pos_(pos), cfg_(cfg) {}

bool StateMachine::begin() {
  normalTiltDeg_ = pos_.config().maxTiltDeg;
  fc_.begin();
  return sens_.begin();
}

const char* StateMachine::stateName() const {
  static const char* const names[] = {"BOOT", "READY", "SPINUP", "CLIMB", "FLY", "SAFETY", "LAND"};
  return names[state_];
}

bool StateMachine::calibrate() {
  heard();
  if (state_ != READY) return false;
  calibrated_ = sens_.startup();                   // blocks 2 s
  now_ = shared::nowUs();                          // fresh time for the FC class after the block
  fc_.update(now_);
  if (calibrated_ && fc_.linkOk()) {               // still on the ground: Betaflight's roll / pitch vs ours
    const drone::FcAttitude a = fc_.attitude();
    const sensors::Attitude n = sens_.attitude();
    refRollOff_  = n.roll  - a.rollDeg  * SM_DEG;
    refPitchOff_ = n.pitch - a.pitchDeg * SM_DEG;
    refOk_ = true;
  }
  heard();
  return calibrated_;
}

bool StateMachine::start() {
  heard();
  if (state_ != READY || !calibrated_) return false;
  calibrated_ = false; go(SPINUP);
  return true;
}
void StateMachine::land() { heard(); if (state_ == SPINUP || state_ == CLIMB || state_ == FLY) go(LAND); }
void StateMachine::kill() { heard(); fc_.disarm(); if (state_ != BOOT) go(READY); }
void StateMachine::setTarget(const sensors::Pose& target, const sensors::Pose& vel, const sensors::Pose& acc) { target_ = target; tVel_ = vel; tAcc_ = acc; }

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
      thr_ = cfg_.spinupFrom * pos_.config().throttleHover;
      break;

    case CLIMB:                                    
      pos_.reset();
      sp_ = p;                                     
      break;

    case FLY:                                      
      origin_ = sp_;
      break;

    case SAFETY:
    case LAND:                                     
      sp_.x = p.x;  sp_.y = p.y;  sp_.z = p.z;
      break;

    default: break;
  }
  spVel_ = spAcc_ = sensors::Pose();
  if (s == READY || s == SPINUP) xyOn_ = false;
  if ((s == SAFETY) != (state_ == SAFETY)) pos_.config().maxTiltDeg = s == SAFETY ? cfg_.safetyTiltDeg : normalTiltDeg_;
  state_ = s; enterUs_ = now_; enterZ_ = p.z; heardUs_ = now_;
}


void StateMachine::step(uint32_t nowUs) {
  dt_ = now_ ? (int32_t)(nowUs - now_) * 1e-6f : 0.0f;
  now_ = nowUs; fc_.update(nowUs);
  const sensors::Pose p = sens_.pose();
  const float v = fc_.batteryV();
  if (cfg_.hoverThrottleV > 0.0f && v > 5.0f) {    // hover throttle from the battery (0 V: not read yet)
    vBat_ = vBat_ > 0.0f ? vBat_ + (v - vBat_) * fminf(dt_ / cfg_.batteryTauS, 1.0f) : v;
    pos_.config().throttleHover = cfg_.hoverThrottleV / vBat_;
  }
  const float hover = pos_.config().throttleHover;

  switch (state_) {
    case BOOT:
      if (fc_.linkReady() && fc_.setupModes(true)) go(READY);
      break;

    case READY:
      break;

    case SPINUP:                                   
      if ((uint32_t)(now_ - enterUs_) > (uint32_t)(cfg_.spinupMaxS * 1e6f)) { go(LAND); break; }   // never lifted off
      if (!fc_.armed()) { thr_ = cfg_.spinupFrom * hover; break; }
      thr_ = fminf(thr_ + cfg_.spinupRate * dt_, cfg_.spinupMax * hover);
      if (p.z > enterZ_ + cfg_.liftoffM) go(CLIMB);
      break;

    case CLIMB: {                                  // setpoint z towards the target's z at climbRate
      const float tz = fminf(target_.z, cfg_.maxHeightM), step = cfg_.climbRate * dt_;
      const float d = smClamp(tz - sp_.z, -step, step);
      sp_.z += d;
      spVel_.z = dt_ > 0.0f ? d / dt_ : 0.0f;
      if (sp_.z == tz && fabsf(p.z - tz) < cfg_.reachedM) go(FLY);   // the setpoint got there, and so did the drone
      break;
    }

    case FLY: {                                    // setpoint towards the target, in the box, at most maxSpeed
      sensors::Pose t = target_;
      t.x = smClamp(t.x, origin_.x - cfg_.maxDistM, origin_.x + cfg_.maxDistM);
      t.y = smClamp(t.y, origin_.y - cfg_.maxDistM, origin_.y + cfg_.maxDistM);
      t.z = smClamp(t.z, 0.0f, cfg_.maxHeightM);
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

    case SAFETY:
    case LAND:
      sp_.z -= downRate() * dt_;
      spVel_.z = -downRate();
      if ((uint32_t)(now_ - enterUs_) > (uint32_t)((fmaxf(enterZ_, 0.0f) / downRate() + cfg_.landExtraS) * 1e6f)) go(READY);
      break;
  }

  const bool lost = cfg_.linkTimeoutMs && (uint32_t)(now_ - heardUs_) > cfg_.linkTimeoutMs * 1000UL;
  if (flying() && !fc_.armed() && !fc_.arming()) go(READY); 
  if (lost && (state_ == SPINUP || state_ == CLIMB || state_ == FLY)) go(SAFETY);
  sens_.flying(flying());                          // height from the ToF while the motors shake the accelerometer
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
  if ((state_ == LAND || state_ == SAFETY) && sp_.z < cfg_.levelBelowM) xyOn_ = false;

  cmd_ = pos_.update(p, sens_.poseVelocity(), sp_, dt_, spVel_, spAcc_);
  if (state_ == CLIMB) cmd_.throttle = fminf(cmd_.throttle, cfg_.climbThrottleMax * pos_.config().throttleHover);
  if (!xyOn_) {
    cmd_.rollDeg = cmd_.pitchDeg = 0.0f;
    pos_.resetXY();
  }
  fc_.setAttitude(cmd_.rollDeg, cmd_.pitchDeg, cmd_.yawRateDps, cmd_.throttle);
}

}  // namespace ctrl
