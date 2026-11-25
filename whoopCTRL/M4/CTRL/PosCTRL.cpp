#include "headers/PosCTRL.h"

#include <math.h>

namespace ctrl {

// 'pc' prefix: sketches include several .cpp files into one compile unit
static const float PC_DEG = 0.01745329f;           // rad per degree

static float pcClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static sensors::Pose pcLimit(const sensors::Pose& a, const sensors::Pose& lim) {   // each axis to +-lim
  return {pcClamp(a.x, -lim.x, lim.x), pcClamp(a.y, -lim.y, lim.y), pcClamp(a.z, -lim.z, lim.z), pcClamp(a.yaw, -lim.yaw, lim.yaw)};
}

static sensors::Pose pcToBody(const sensors::Pose& w, float yaw) {   // x / y turned by -yaw; z, yaw as they are
  const float c = cosf(yaw), s = sinf(yaw);
  return {c * w.x + s * w.y, -s * w.x + c * w.y, w.z, w.yaw};
}

PosCTRL::Command PosCTRL::update(const sensors::Pose& p, const sensors::Pose& vel, const sensors::Pose& target, float dt,const sensors::Pose& targetVel, const sensors::Pose& targetAcc) {
  return translate(controller(p, vel, target, dt, targetVel, targetAcc));
}

PosCTRL::LinCMD PosCTRL::controller(const sensors::Pose& p, const sensors::Pose& vel, const sensors::Pose& target, float dt, const sensors::Pose& targetVel, const sensors::Pose& targetAcc) {
  sensors::Pose err = target - p;                                 
  err.yaw = sensors::wrapPi(err.yaw);                              
  const sensors::Pose errDot = targetVel - vel;
  sensors::Pose kp = cfg_.kp, kd = cfg_.kd, ki = cfg_.ki;
  if (climb_) { kp.z *= cfg_.climbKpScale;  kd.z *= cfg_.climbKdScale;  ki.z *= cfg_.climbKiScale; }
  if (dt > 0.0f && dt < 0.5f) {                                    // the integral keeps its value
    sensors::Pose add = ki * err * dt;                             // per axis: frozen while that setpoint moves
    if (fabsf(targetVel.x) > cfg_.iFreezeVel) add.x = 0.0f;        // (it added up during the move and carried
    if (fabsf(targetVel.y) > cfg_.iFreezeVel) add.y = 0.0f;        // the drone past the end)
    if (fabsf(targetVel.z) > cfg_.iFreezeVel) add.z = 0.0f;
    if (fabsf(targetVel.yaw) > cfg_.iFreezeVel) add.yaw = 0.0f;
    integ_ = pcLimit(integ_ + add, cfg_.iLimit);
  }

  const sensors::Pose nu = targetAcc + kp * err + kd * errDot + integ_;   // wanted acceleration, m/s^2 (a learned
  const sensors::Pose u  =(pcToBody(nu, p.yaw) + cfg_.f2 * pcToBody(vel, p.yaw)) / cfg_.f1;   // correction: nu.z - rHat)
  return {u.x, u.y, u.z, u.yaw};                                    // U = f1^-1 (nu + f2 X'); z: throttle change from hover
}

PosCTRL::Command PosCTRL::translate(const LinCMD& u) const {
  const float maxTilt = cfg_.maxTiltDeg * PC_DEG;
  const float pitch = pcClamp(u.vX, -maxTilt, maxTilt);           
  const float roll  = pcClamp(u.vY, -maxTilt, maxTilt);           
  Command c;
  c.pitchDeg   = cfg_.pitchSign * pitch / PC_DEG;
  c.rollDeg    = cfg_.rollSign * roll / PC_DEG;
  c.yawRateDps = cfg_.yawSign * pcClamp(u.yawRate / PC_DEG, -cfg_.maxYawRateDps, cfg_.maxYawRateDps);
  
  c.throttle = pcClamp((cfg_.throttleHover + u.vZ) / (cosf(pitch) * cosf(roll)), cfg_.throttleMin, cfg_.throttleMax);
  return c;
}

}  
