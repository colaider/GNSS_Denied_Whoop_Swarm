#pragma once

#include <stdint.h>

#include "PosCTRL.h"
#include "../../SENSORS/headers/SENSORS.h"
#include "../../DrCTRL/headers/betaflight.h"

namespace ctrl {

// How to fly, never where: the target comes from setTarget() every loop.
struct SMConfig {
  float    hoverThrottleV = 2.59f;  // hover throttle x battery volts (logs 11 + 17.11: 2.59 +-0.01 from 6.7 to 7.8 V):
                                    // sets PosConfig::throttleHover every loop (0: keep the fixed one). Fixed 0.34:
                                    // 8 % too much on a full battery, shot up at takeoff
  float    batteryTauS   = 2.0f;    // battery volts smoothed over this
  float    spinupFrom    = 0.92f;   // SPINUP: throttle starts at this x hover throttle, level, no feedback...
  float    spinupRate    = 0.08f;   // ...rises this much per second...
  float    spinupMax     = 1.10f;   // ...up to this x hover throttle (liftoff is seen late: it ran on to 0.40, +25 %)
  float    spinupMaxS    = 5.0f;    // SPINUP longer than this (arming included), never lifted off: LAND
  float    liftoffM      = 0.03f;   // SPINUP -> CLIMB this far above the start height
  float    climbRate     = 0.3f;    // m/s, CLIMB: setpoint z rises to the target's z
  float    climbThrottleMax = 1.15f;   // CLIMB: throttle at most this x hover throttle (a wrong height
                                       // reading pushed it to 0.41 = 1.3x, it shot up to 97 cm, log 17.11)
  float    reachedM      = 0.03f;   // CLIMB -> FLY within this of the target's z
  float    maxSpeed      = 0.5f;    // m/s, FLY: the setpoint follows the target at most this fast
  float    maxDistM      = 1.5f;    // FLY: setpoint x / y kept within this of the origin
  float    maxHeightM    = 1.5f;    // setpoint z kept below this
  float    descendRate   = 0.25f;   // m/s, LAND
  float    safetyRate    = 0.15f;   // m/s, SAFETY
  float    safetyTiltDeg = 5.0f;    // SAFETY: tilt limit (normal: PosConfig::maxTiltDeg)
  float    landExtraS    = 2.0f;    // LAND / SAFETY disarm after (height / rate + this): by time, not ToF
  uint32_t linkTimeoutMs = 500;     // no heard() this long while flying -> SAFETY (0 = off)
  float    xyAfterS      = 0.5f;    // x / y control starts after the camera tracked this long (level before)
  float    levelBelowM   = 0.15f;   // LAND / SAFETY: setpoint lower than this -> level only (near the floor the camera
                                    // velocity is wrong: x / y ran 20 cm, 5 deg tilt, crashed, log 11.11)
};


class StateMachine {
public:
    enum State : uint8_t { BOOT, READY, SPINUP, CLIMB, FLY, SAFETY, LAND };

    StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg = SMConfig());

    bool begin();                    // FC UART + sensors (calibration: calibrate())
    void step(uint32_t nowUs);       // every loop: the whole state machine

    bool calibrate();                // READY, drone still 2 s: gyro bias + start frame (position 0). false: moved
    bool start();                    // READY + calibrated: arm, take off. Each flight needs a new calibrate()
    void land();
    void kill();
    // Where to be, start frame (calibration point on the floor = 0). Call every loop. CLIMB uses its z, FLY all of it.
    void setTarget(const sensors::Pose& target, const sensors::Pose& vel = sensors::Pose(), const sensors::Pose& acc = sensors::Pose());
    void heard() { heardUs_ = now_; }  // the PC is there

    State       state() const { return state_; }
    const char* stateName() const;
    bool        calibrated() const { return calibrated_; }
    const sensors::Pose& setpoint() const { return sp_; }     // what the controller follows now
    const sensors::Pose& origin() const { return origin_; }   // where the hover was reached (trajectories start here)
    const PosCTRL::Command& command() const { return cmd_; }  // the last one sent to the FC (0 when not flying)
    bool        xyControl() const { return xyOn_; }           // x / y held (false: level, drifting)
    SMConfig&   config() { return cfg_; }

private:
    void go(State s);
    void control();
    bool flying() const { return state_ >= SPINUP; }
    float downRate() const { return state_ == SAFETY ? cfg_.safetyRate : cfg_.descendRate; }

    sensors::Sensors&  sens_;
    drone::Betaflight& fc_;
    PosCTRL&           pos_;
    SMConfig           cfg_;

    State         state_ = BOOT;
    uint32_t      now_ = 0, enterUs_ = 0, heardUs_ = 0, camBadUs_ = 0;
    float         dt_ = 0.0f, enterZ_ = 0.0f, thr_ = 0.0f, vBat_ = 0.0f;
    bool          calibrated_ = false, xyOn_ = false;
    float         normalTiltDeg_ = 15.0f;
    sensors::Pose target_, tVel_, tAcc_;   // from setTarget()
    sensors::Pose sp_, spVel_, spAcc_;     // setpoint
    sensors::Pose origin_;
    PosCTRL::Command cmd_{};
};

}
