#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"

namespace ctrl {

struct PosConfig {
  sensors::Pose f1{9.81f, 9.81f, 60.0f, 8.0f};    
  sensors::Pose f2{0.4f, 0.4f, 0.5f, 0.0f};       // yaw 0: f2/f1 = 1 passed the gyro noise straight to the FC (+-20 deg/s on the ground, FC disarmed)       
  // The drone's gains (x y z yaw). x / y: 4 / 3.2 / 0.8 flew the forward-back trajectory well (17.11: x err
  // 1.6 cm, y 1.2), autoTune went higher on 8 cm steps (x 5 / 3.2 / 1.0, y 6.25 / 4.0 / 1.25: noisy, the drone
  // is symmetric): between, kd / sqrt(kp) kept (damping). z: 3 / 3.5 / 1.0 (4.5: swung at 1.7 s; the step
  // trials differed within noise). autoTune.py starts from these.
  sensors::Pose kp{4.5f, 4.5f, 3.0f, 3.0f};
  sensors::Pose kd{3.4f, 3.4f, 3.5f, 0.0f};
  sensors::Pose ki{0.9f, 0.9f, 1.0f, 0.0f};
  sensors::Pose iLimit{1.0f, 1.0f, 3.0f, 0.5f};

  float throttleHover = 0.34f;    // until the battery is read: then 2.59 / V (SMConfig::hoverThrottleV)
  float throttleMin   = 0.05f;
  float throttleMax   = 0.45f;
  float maxTiltDeg    = 8.0f;
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
    void resetXY() { integ_.x = integ_.y = 0.0f; }
    const sensors::Pose& integral() const { return integ_; }     
    PosConfig& config() { return cfg_; }           

private:
    PosConfig     cfg_;
    sensors::Pose integ_;                         
};

} 
