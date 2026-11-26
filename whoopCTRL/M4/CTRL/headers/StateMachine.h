#pragma once

#include <stdint.h>

#include "PosCTRL.h"
#include "../../SENSORS/headers/SENSORS.h"
#include "../../DrCTRL/headers/betaflight.h"

namespace ctrl {

// How to fly, never where: the target comes from setTarget() every loop.
struct SMConfig {
  float    batteryTauS   = 2.0f;    // battery volts smoothed over this
  float    spinupS       = 1.5f;    // SPINUP: throttle 0 -> hover throttle in this long (after arming), level, no
                                    // feedback; then CLIMB. To 1.10x it lifted off on its own and shot up (log 23.11)
  float    climbRate     = 0.3f;    // m/s, CLIMB: peak speed of the setpoint's smooth (min-jerk) rise to the target z;
                                    // speed and acceleration 0 at both ends: no jerk. Arrived -> FLY
  float    climbLeadS    = 0.1f;    // s, CLIMB: its acceleration (feed-forward) sent this far ahead, the thrust
                                    // arrives that much later (as Trajectory::leadZS). 0: off
  float    maxSpeed      = 0.5f;    // m/s, FLY: the setpoint follows the target at most this fast
  float    maxDistM      = 1.5f;    // FLY: setpoint x / y kept within this of the origin
  float    maxHeightM    = 1.5f;    // setpoint z kept below this
  float    descendRate   = 0.25f;   // m/s, LAND
  float    landExtraS    = 2.0f;    // LAND: disarm after (height / rate + this): by time, not ToF
  uint32_t linkTimeoutMs = 500;     // no heard() this long while flying -> LAND (0 = off)
  float    xyAfterS      = 0.5f;    // x / y control starts after the camera tracked this long (level before)
  float    levelBelowM   = 0.15f;   // LAND: setpoint lower than this -> level only (near the floor the camera
                                    // velocity is wrong: x / y ran 20 cm, 5 deg tilt, crashed, log 11.11)
};


class StateMachine {
public:
    enum State : uint8_t { BOOT, READY, SPINUP, CLIMB, FLY, LAND = 6 };   // 5 was SAFETY (now LAND): old logs read the same

    StateMachine(sensors::Sensors& sens, drone::Betaflight& fc, PosCTRL& pos, const SMConfig& cfg = SMConfig());

    bool begin();                    // FC UART + sensors (calibration: calibrate())
    void step(uint32_t nowUs);       // every loop: the whole state machine

    bool calibrate();                // READY, drone still 2 s: gyro bias + start frame (position 0). false: moved
    bool start();                    // READY + calibrated + a known Nicla (drones::known()): arm, take off.
                                     // Each flight needs a new calibrate()
    void land();
    void kill();
    // Where to be, start frame (calibration point on the floor = 0). Call every loop. CLIMB uses its z, FLY all of it.
    void setTarget(const sensors::Pose& target, const sensors::Pose& vel = sensors::Pose(), const sensors::Pose& acc = sensors::Pose());
    void heard() { heardUs_ = now_; }  // the PC is there

    State       state() const { return state_; }
    const char* stateName() const;
    bool        calibrated() const { return calibrated_; }
    bool        referenceOk() const { return refOk_; }      // Betaflight's roll / pitch offset taken (start() needs it)
    const sensors::Pose& setpoint() const { return sp_; }     // what the controller follows now
    const sensors::Pose& origin() const { return origin_; }   // where the hover was reached (trajectories start here)
    const PosCTRL::Command& command() const { return cmd_; }  // the last one sent to the FC (0 when not flying)
    bool        xyControl() const { return xyOn_; }           // x / y held (false: level, drifting)
    SMConfig&   config() { return cfg_; }

private:
    void go(State s);
    void takeReference();
    void control();
    bool flying() const { return state_ >= SPINUP; }
    sensors::Pose limited(sensors::Pose t) const;   // the target inside the box / below maxHeightM

    sensors::Sensors&  sens_;
    drone::Betaflight& fc_;
    PosCTRL&           pos_;
    SMConfig           cfg_;

    State         state_ = BOOT;
    uint32_t      now_ = 0, enterUs_ = 0, heardUs_ = 0, camBadUs_ = 0, armedUs_ = 0;
    float         dt_ = 0.0f, enterZ_ = 0.0f, thr_ = 0.0f, vBat_ = 0.0f;
    float         climbFrom_ = 0.0f, climbTo_ = 0.0f, climbS_ = 0.0f;   // CLIMB: z from, to, duration
    float         refRollOff_ = 0.0f, refPitchOff_ = 0.0f;   // ours - Betaflight's roll / pitch, rad (takeReference())
    bool          refOk_ = false;
    bool          calibrated_ = false, xyOn_ = false;
    sensors::Pose target_, tVel_, tAcc_;   // from setTarget()
    sensors::Pose sp_, spVel_, spAcc_;     // setpoint
    sensors::Pose origin_;
    PosCTRL::Command cmd_{};
};

}
