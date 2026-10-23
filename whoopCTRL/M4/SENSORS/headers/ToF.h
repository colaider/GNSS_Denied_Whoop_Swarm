// ToF - the VL53L1CB distance sensor (pointing down) on the M4, read without waiting.
//
// It measures continuously (default: short mode, 20 ms per measurement = 50 Hz). update()
// only reads when a new measurement is ready, so it never blocks the control loop.
// Each measurement is stamped with the shared clock at the MIDDLE of its integration time
// (when the distance was measured), not when it was read.
//
// Height above the floor: the beam tilts with the drone, so h = range * cos(roll) * cos(pitch).
//
//   sensors::ToF tof;                         // global
//   tof.begin();
//   if (tof.update() && tof.last().valid)     // a new, trusted measurement
//     float h = sensors::ToF::height(tof.last().rangeM, att.roll, att.pitch);
//   if (tof.validAgeUs(shared::nowUs()) > 200000) { ... }   // no valid height for 200 ms
//
// Needs the Pololu "VL53L1X" library. No heap memory, nothing to free.

#pragma once

#include <stdint.h>

namespace sensors {

struct ToFConfig {
  bool     longMode  = false;      // false: short mode (to ~1.3 m, best in daylight); true: ~4 m
  uint32_t budgetUs  = 20000;      // time per measurement (short >= 20 ms, long >= 33 ms)
  float    minRangeM = 0.04f;      // closer than this is not trusted
  float    maxRangeM = 1.3f;       // farther than this is not trusted
  float    minSignal = 0.5f;       // weaker return (MCPS) is not trusted: dark or far floor
};

struct ToFReading {
  uint32_t tUs         = 0;        // shared clock, middle of the measurement
  float    rangeM      = 0.0f;     // distance along the beam, metres
  bool     valid       = false;    // sensor status ok, inside min/max range, strong enough
  uint8_t  status      = 255;      // the sensor's range status (0 = valid)
  float    signalMcps  = 0.0f;     // return signal strength: higher = more trustworthy
  float    ambientMcps = 0.0f;     // background light
};

class ToF {
 public:
  explicit ToF(const ToFConfig& cfg = ToFConfig());

  // Internal I2C bus (Wire1, 400 kHz), sensor setup, starts measuring. False if no sensor.
  bool begin();

  // Reads a new measurement if one is ready; never waits. True if it read one.
  bool update();

  const ToFReading& last() const { return last_; }            // newest, valid or not
  const ToFReading& lastValid() const { return lastValid_; }  // newest valid one

  // How old the newest valid measurement is (0xFFFFFFFF if there never was one).
  uint32_t validAgeUs(uint32_t nowUs) const;
  uint32_t timeouts() const { return timeouts_; }             // I2C timeouts so far

  // Height above a flat floor from a range measured with the drone tilted (radians).
  static float height(float rangeM, float roll, float pitch);

  // Raw sensor values -> a reading (timestamp, units, validity). Public for testing.
  ToFReading make(uint32_t readyUs, uint16_t rangeMm, uint8_t status, float signalMcps, float ambientMcps) const;

 private:
  ToFConfig  cfg_;
  ToFReading last_, lastValid_;
  bool       haveValid_ = false;
  uint32_t   timeouts_  = 0;
};

}  // namespace sensors
