#include "headers/StateMachine.h"

#include <math.h>

#include "../../shared/headers/sharedClock.h"

namespace ctrl {

StateMachine::StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg) : sens_(sens), fc_(fc), pos_(pos), cfg_(cfg) {}

bool StateMachine::begin() {
  normalTiltDeg_ = pos_.config().maxTiltDeg;
  fc_.begin();
  return sens_.begin();
}


const char* StateMachine::stateName() const {
  static const char* const names[] = {"BOOT", "READY", "TAKEOFF", "FLY", "SAFETY", "LAND"};
  return names[state_];
}

bool StateMachine::calibrate() {
  heard();
  if (state_ != READY) return false;
  calibrated_ = sens_.startup();                   // blocks 2 s
  now_ = shared::nowUs();                          // fresh time for the FC class after the block
  fc_.update(now_);
  heard();
  return calibrated_;
}

bool StateMachine::start() {
  heard();
  if (state_ != READY || !calibrated_) return false;
  calibrated_ = false;                             // the next flight calibrates again
  go(TAKEOFF);
  return true;
}
void StateMachine::land() { heard(); if (state_ == TAKEOFF || state_ == FLY) go(LAND); }
void StateMachine::kill() { heard(); fc_.disarm(); if (state_ != BOOT) go(READY); }
void StateMachine::setTarget(const sensors::Pose& target, const sensors::Pose& vel, const sensors::Pose& acc) { heard(); if (state_ != FLY) return; target_ = target; tVel_ = vel; tAcc_ = acc; }

void StateMachine::go(State s) {
  const sensors::Pose p = sens_.pose();
  switch (s) {
    case READY:
      fc_.disarm();
      break;

    case TAKEOFF:                                  // setpoint: above where it stands (x 0, y 0, yaw 0 after calibration)
      pos_.reset();
      target_ = p;
      target_.z = cfg_.hoverHeight;
      fc_.arm();
      break;

    case SAFETY:

    case LAND:                                     // setpoint start: where it is now, yaw kept
      target_.x = p.x;
      target_.y = p.y;
      target_.z = p.z;
      break;
   
    default: break;
  }

  tVel_ = sensors::Pose(); tAcc_ = sensors::Pose();
  if (s == READY) cmd_ = PosCTRL::Command{};
  if (s == READY || s == TAKEOFF) xyOn_ = false;
  
  if (s == SAFETY && state_ != SAFETY) {
    normalTiltDeg_ = pos_.config().maxTiltDeg;
    pos_.config().maxTiltDeg = cfg_.safetyTiltDeg;

  } else if (s != SAFETY && state_ == SAFETY) {
    pos_.config().maxTiltDeg = normalTiltDeg_;
  }

  state_ = s; enterUs_ = now_; enterZ_ = p.z; heardUs_ = now_;                                
}


void StateMachine::step(uint32_t nowUs) {
  const float dt = now_ ? (int32_t)(nowUs - now_) * 1e-6f : 0.0f;
  now_ = nowUs;
  fc_.update(nowUs);

  if (!transitions()) return;      // 1. change state if needed. false: motors off, nothing to control
  setpoint(dt);                    // 2. where the drone should be now: target_, tVel_
  control(dt);                     // 3. controller -> FC
}

// 1. state changes only
bool StateMachine::transitions() {
  const bool flying = state_ == TAKEOFF || state_ == FLY || state_ == SAFETY || state_ == LAND;
  if (flying && !fc_.armed() && !fc_.arming()) { go(READY); return false; }
  const bool lost = cfg_.linkTimeoutMs && (uint32_t)(now_ - heardUs_) > cfg_.linkTimeoutMs * 1000UL;

  switch (state_) {
    case BOOT:
      if (fc_.linkReady() && fc_.setupModes(true)) go(READY);
      return false;

    case READY:
      return false;

    case TAKEOFF:
      if (!fc_.armed()) return false;              // arm sequence running: the FC class holds throttle low
      if (lost) go(SAFETY);
      else if (fabsf(sens_.pose().z - cfg_.hoverHeight) < cfg_.reachedM) go(FLY);
      return true;

    case FLY:
      if (lost) go(SAFETY);
      return true;

    case SAFETY:
    case LAND: {
      const float downS = fmaxf(enterZ_, 0.0f) / downRate() + cfg_.landExtraS;   // by time: ToF is unreliable near the floor
      if ((uint32_t)(now_ - enterUs_) > (uint32_t)(downS * 1e6f)) { go(READY); return false; }
      return true;
    }
  }
  return false;
}

// 2. the setpoint. Set in go(): TAKEOFF = hoverHeight above the start, SAFETY / LAND = where it is.
// Only LAND / SAFETY move it (down); TAKEOFF / FLY keep it (or setTarget() in FLY)
void StateMachine::setpoint(float dt) {
  if (state_ == SAFETY || state_ == LAND) {        // go down, x / y / yaw held
    target_.z -= downRate() * dt;
    tVel_.z = -downRate();
  }
}

// 3. the controller. x / y only once the camera has tracked xyAfterS in a row (near the floor it sees
// nothing, logs 03.11: under 10 cm ~1-25% of frames); x / y target = where it is then. Before, and
// whenever the camera is LOST: roll / pitch 0, Betaflight keeps it level, height control only.
void StateMachine::control(float dt) {
  const sensors::Pose p = sens_.pose();
  if (sens_.horizontalFix() != sensors::Fix::TRACKING) camBadUs_ = now_;
  if (!xyOn_ && (uint32_t)(now_ - camBadUs_) >= (uint32_t)(cfg_.xyAfterS * 1e6f)) {
    xyOn_ = true;
    target_.x = p.x;  target_.y = p.y;
  }
  if (sens_.horizontalFix() == sensors::Fix::LOST) xyOn_ = false;

  cmd_ = pos_.update(p, sens_.poseVelocity(), target_, dt, tVel_, tAcc_);
  if (!xyOn_) {
    cmd_.rollDeg = cmd_.pitchDeg = 0.0f;
    pos_.resetXY();
  }
  fc_.setAttitude(cmd_.rollDeg, cmd_.pitchDeg, cmd_.yawRateDps, cmd_.throttle);
}

}  // namespace ctrl
