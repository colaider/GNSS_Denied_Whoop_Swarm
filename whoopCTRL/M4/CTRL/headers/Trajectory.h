#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"
#include "../../../shared/headers/messages.h"

namespace ctrl {

// Plays a trajectory sent by the PC as points (msg::TrajPoint: time from the start, position, velocity).
// Only a clock and the points: sample() says where the curve is now (cubic Hermite between the two points
// around that time), or false when there is no point around now (not started yet, ran out, cleared).
// No states, no holding: what to do without a point is the sketch's business.
// Positions are relative to StateMachine::origin.
class Trajectory {
public:
    // Acceleration (the feed-forward) taken this far ahead: Betaflight reaches a commanded tilt ~0.2 s
    // late (log 17.11 15:26, 96-98 % of it), so the moves lagged 4-6 cm and overshot 5-6 cm.
    // Simulated with that delay: overshoot 2.7 -> 0.4 cm. 0: off.
    float leadS = 0.2f;             // x / y (tilt)
    // z: the thrust follows the throttle ~0.1 s late (logs 22-24.11: best fit 80-120 ms; MSP wait, Betaflight
    // RC smoothing ~13 ms, motor spin-up), so z with 0.2 started climbing ~0.1 s too early. 0: off.
    float leadZS = 0.1f;

    bool start(uint16_t id, uint32_t startUs);                 // t = 0 at startUs, old points dropped. False: same id again
    void add(uint16_t id, const msg::TrajPoint* pts, int n);   // points of the started id; repeats replace, any order
    void clear() { n_ = 0;  id_ = 0; }                         // forget everything (HOLD, not flying)
    bool sample(uint32_t nowUs, sensors::Pose& p, sensors::Pose& v, sensors::Pose& a);   // false: no point around now

private:
    void drop();                                               // forget the oldest point
    bool eval(float t, sensors::Pose& p, sensors::Pose& v, sensors::Pose& acc) const;   // the curve at time t; false outside the points

    static constexpr int MAX_POINTS = 32;
    msg::TrajPoint pts_[MAX_POINTS];
    int           n_ = 0;
    uint16_t      id_ = 0;                                     // 0: none (the PC counts from 1)
    uint32_t      startUs_ = 0;
};

}
