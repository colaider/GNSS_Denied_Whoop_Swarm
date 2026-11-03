#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"

namespace ctrl {

struct PosConfig {
  sensors::Pose f1{9.81f, 9.81f, 50.0f, 8.0f};    
  sensors::Pose f2{0.4f, 0.4f, 0.5f, 0.0f};       // yaw 0: f2/f1 = 1 passed the gyro noise straight to the FC (+-20 deg/s on the ground, FC disarmed)       
  sensors::Pose kp{0.0f, 0.0f, 0.1f, 0.0f};     
  sensors::Pose kd{0.0f, 0.0f, 0.0f, 0.0f};       
  sensors::Pose ki{0.0f, 0.0f, 0.0f, 0.0f};         
  sensors::Pose iLimit{1.0f, 1.0f, 3.0f, 0.5f};

  float throttleHover = 0.25f;    
  float throttleMin   = 0.05f;   
  float throttleMax   = 0.55f;
  float maxTiltDeg    = 10.0f;    
  float maxYawRateDps = 90.0f;
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
