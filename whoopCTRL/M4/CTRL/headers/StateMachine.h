#pragma once

#include <stdint.h>

#include "PosCTRL.h"
#include "../../SENSORS/headers/SENSORS.h"
#include "../../DrCTRL/headers/betaflight.h"

namespace ctrl {

struct SMConfig {
  float    hoverHeight   = 0.1f;    // m, TAKEOFF setpoint
  float    reachedM      = 0.05f;   // TAKEOFF -> FLY within this of hoverHeight
  float    descendRate   = 0.25f;   // m/s, LAND
  float    safetyRate    = 0.15f;   // m/s, SAFETY
  float    safetyTiltDeg = 5.0f;    // SAFETY: tilt limit (normal: PosConfig::maxTiltDeg)
  float    landExtraS    = 2.0f;    // LAND / SAFETY disarm after (height / rate + this): by time, not ToF
  uint32_t linkTimeoutMs = 500;     // no heard() / command this long while flying -> SAFETY (0 = off)
  float    xyAfterS      = 0.5f;    // x / y control starts after the camera tracked this long (level before)
};

class StateMachine {
public:
    enum State : uint8_t { BOOT, READY, TAKEOFF, FLY, SAFETY, LAND };

    StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg = SMConfig());

    bool begin();                    // FC UART + sensors (calibration: calibrate())
    void step(uint32_t nowUs);       

    bool calibrate();                // READY, drone still 2 s: gyro bias + start frame (position 0). false: moved
    bool start();                    // READY + calibrated: arm, take off. Each flight needs a new calibrate()
    bool calibrated() const { return calibrated_; }
    void land();                     
    void kill();             
    void setTarget(const sensors::Pose& target, const sensors::Pose& vel = sensors::Pose(), const sensors::Pose& acc = sensors::Pose()); 
    void heard() { heardUs_ = now_; }

    State       state() const { return state_; }
    const char* stateName() const;
    const sensors::Pose& target() const { return target_; }
    const PosCTRL::Command& command() const { return cmd_; }   // the last one sent to the FC (0 when not flying)
    bool        xyControl() const { return xyOn_; }            // x / y held (false: level, drifting)
    SMConfig&   config() { return cfg_; }

private:
    void go(State s);
    bool transitions();
    void setpoint(float dt);
    void control(float dt);
    float downRate() const { return state_ == SAFETY ? cfg_.safetyRate : cfg_.descendRate; }

    sensors::Sensors&  sens_;
    drone::Betaflight& fc_;
    PosCTRL&           pos_;
    SMConfig           cfg_;

    State         state_ = BOOT;
    uint32_t      now_ = 0, enterUs_ = 0, heardUs_ = 0;
    float         enterZ_ = 0.0f;
    bool          calibrated_ = false;
    bool          xyOn_ = false;
    uint32_t      camBadUs_ = 0;      // last time the camera was not tracking
    float         normalTiltDeg_ = 15.0f;
    sensors::Pose target_, tVel_, tAcc_;
    PosCTRL::Command cmd_{};
};

} 
