#pragma once

#include <stdint.h>

#include "../../SENSORS/headers/SENSORS.h"
#include "../../../shared/headers/messages.h"

namespace ctrl {

// Plays a trajectory sent by the PC as points (msg::TrajPoint: time from the start, position, velocity).
// One clock: every update() asks "where now", cubic Hermite between the two points around that time.
// Positions are relative: the sketch adds them to where the trajectory starts (StateMachine::origin).
class Trajectory {
public:
    enum State : uint8_t { IDLE, WAITING, PLAYING, EMPTY, HOLD };

    // Acceleration (the feed-forward) taken this far ahead: Betaflight reaches a commanded tilt ~0.2 s
    // late (log 17.11 15:26, 96-98 % of it), so the moves lagged 4-6 cm and overshot 5-6 cm.
    // Simulated with that delay: overshoot 2.7 -> 0.4 cm. 0: off.
    float leadS = 0.2f;

    void start(uint16_t id, uint32_t startUs);                 // t = 0 at startUs, old points dropped. Same id again: ignored
    void add(uint16_t id, const msg::TrajPoint* pts, int n);   // points of the started id; repeats replace, any order
    void hold() { if (state_ != IDLE) state_ = HOLD; }          // stop where it is now
    void clear() { state_ = IDLE; n_ = 0; p_ = v_ = a_ = sensors::Pose(); }
    bool update(uint32_t nowUs);                               // false: IDLE, nothing to follow

    const sensors::Pose& position() const { return p_; }
    const sensors::Pose& velocity() const { return v_; }
    const sensors::Pose& acceleration() const { return a_; }
    State       state() const { return state_; }
    const char* stateName() const;
    uint16_t    id() const { return id_; }
    int         points() const { return n_; }

private:
    void drop();                                               // forget the oldest point
    bool eval(float t, sensors::Pose& p, sensors::Pose& v, sensors::Pose& acc) const;   // the curve at time t; false outside the points

    static constexpr int MAX_POINTS = 32;
    msg::TrajPoint pts_[MAX_POINTS];
    int           n_ = 0;
    uint16_t      id_ = 0;
    uint32_t      startUs_ = 0;
    State         state_ = IDLE;
    sensors::Pose p_, v_, a_;
};

}
