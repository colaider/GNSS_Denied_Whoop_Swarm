#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"
#include "../../../drones/drones.h"

namespace ctrl {

inline sensors::Pose pose4(const float v[4]) { return sensors::Pose{v[0], v[1], v[2], v[3]}; }   // x y z yaw

struct PosConfig {
  sensors::Pose f1{9.81f, 9.81f, 27.5f, 8.0f};
  sensors::Pose f2{0.4f, 0.4f, 0.5f, 0.0f};           
  // Gains: per drone in drones/drones.h (setGains() at take-off); these only until then.
  sensors::Pose kp = pose4(drones::DEFAULT.kp), kd = pose4(drones::DEFAULT.kd), ki = pose4(drones::DEFAULT.ki);
  sensors::Pose iLimit{1.0f, 1.0f, 1.38f, 0.5f};
  float iFreezeVel = 0.01f;       // m/s: an axis' integral stands still while its setpoint moves faster than this
  float climbKpScale = 0.7f;
  float climbKdScale = 0.84f;
  float climbKiScale = 0.0f;

  float throttleHover = 0.34f;    // until the battery is read: then Drone::hoverThrottleV / V (StateMachine)
  float throttleMin   = 0.05f;
  float throttleMax   = 0.45f;
  float maxTiltDeg    = 12.0f;   // 8: reached only 0.8 % of the time (log 17.11 15:26); headroom for pushes
  float maxYawRateDps = 20.0f;
  float pitchSign = 1.0f;         
  float rollSign  = -1.0f;       
  float yawSign   = -1.0f;        
};


class PosCTRL {
public:
    struct Command {             
        float rollDeg, pitchDeg;
        float yawRateDps;
        float throttle;          
    };

    struct LinCMD {               
        float vX, vY;            
        float vZ;                
        float yawRate;           
    };

    explicit PosCTRL(const PosConfig& cfg = PosConfig()) : cfg_(cfg) {}

    Command update(const sensors::Pose& p, const sensors::Pose& vel, const sensors::Pose& target, float dt, const sensors::Pose& targetVel = sensors::Pose(), const sensors::Pose& targetAcc = sensors::Pose());
    Command translate(const LinCMD& u) const;        
    LinCMD  controller(const sensors::Pose& p, const sensors::Pose& vel, const sensors::Pose& target, float dt, const sensors::Pose& targetVel, const sensors::Pose& targetAcc);

    void reset() { integ_ = sensors::Pose(); }
    void setGains(const float kp[4], const float kd[4], const float ki[4]) { cfg_.kp = pose4(kp); cfg_.kd = pose4(kd); cfg_.ki = pose4(ki); }
    void climbGains(bool on) { climb_ = on; }       // z: the climb gain set (true) or the hover one
    void resetXY() { integ_.x = integ_.y = 0.0f; }
    const sensors::Pose& integral() const { return integ_; }
    PosConfig& config() { return cfg_; }           

private:
    PosConfig     cfg_;
    sensors::Pose integ_;
    bool          climb_ = false;                 // climbGains()
};

} 
