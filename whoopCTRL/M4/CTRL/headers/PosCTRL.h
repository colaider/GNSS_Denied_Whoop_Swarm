#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"

namespace ctrl {

struct PosConfig {
  sensors::Pose f1{9.81f, 9.81f, 27.5f, 8.0f};
  sensors::Pose f2{0.4f, 0.4f, 0.5f, 0.0f};           
  // 24.11: damped (zeta = kd / (2 sqrt(kp)) = 1.2, was 0.69 z / 0.8 x y), x / y softer (kp 4.5 -> 3), and the
  // integral frozen while the setpoint moves (iFreezeVel). Simulated (20 cm min-jerk move, delay z 0.1 / x y
  // 0.2 s, real gain 20 % off, constant push): overshoot z 1.4 -> 0.6 cm, x / y 1.8 -> 1.1 cm, settled in
  // z 1.9 -> 0.8 s, x / y 1.0 -> 0.7 s. The integral adding up during a move was what carried it past the end.
  // x / y 24.11 22:2x: kp 3 -> 4, kd 4.16 -> 3.4, ki 0.6 -> 0.4. kd 4.16 doubled the fast tilt jitter
  // (pitch 2-5 Hz 0.28 -> 0.57 deg: kd x the camera's speed noise); the slow swing is limited by the tilt lag.
  sensors::Pose kp{4.0f, 4.0f, 2.75f, 3.0f};
  sensors::Pose kd{3.4f, 3.4f, 3.98f, 0.0f};
  // ki z 0.69 -> 0.4: above kp^2 / (4 kd) = 0.47 the slow poles are complex: the integral swung the height
  // 4-6 cm every 10-15 s (log 24.11 21:26, poles -0.40 +- 0.24j); 0.4: all real (-3.15, -0.6, -0.2).
  sensors::Pose ki{0.4f, 0.4f, 0.4f, 0.0f};
  sensors::Pose iLimit{1.0f, 1.0f, 1.38f, 0.5f};
  float iFreezeVel = 0.01f;       // m/s: an axis' integral stands still while its setpoint moves faster than this
  float climbKpScale = 0.7f;
  float climbKdScale = 0.84f;
  float climbKiScale = 0.0f;

  float throttleHover = 0.34f;    // until the battery is read: then 2.59 / V (SMConfig::hoverThrottleV)
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
