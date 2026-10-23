#include "headers/ToF.h"

#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <math.h>

#include "../../shared/headers/sharedClock.h"

namespace sensors {

// One sensor on the board: the Pololu driver object lives here, so the header needs no
// library. ('tof' prefix: sketches include several .cpp files into one compile unit.)
static VL53L1X tofSensor;

ToF::ToF(const ToFConfig& cfg) : cfg_(cfg) {
  const uint32_t minBudget = cfg_.longMode ? 33000 : 20000;   // the sensor's minimums
  if (cfg_.budgetUs < minBudget) cfg_.budgetUs = minBudget;
}

bool ToF::begin() {
  Wire1.begin();                                   // internal bus: ToF, power chip, fuel gauge
  Wire1.setClock(400000);
  tofSensor.setBus(&Wire1);
  tofSensor.setTimeout(100);                       // ms, so a missing sensor cannot hang begin()
  if (!tofSensor.init()) return false;
  tofSensor.setDistanceMode(cfg_.longMode ? VL53L1X::Long : VL53L1X::Short);
  if (!tofSensor.setMeasurementTimingBudget(cfg_.budgetUs)) return false;
  tofSensor.startContinuous(cfg_.budgetUs / 1000); // a new measurement every budget
  return true;
}

bool ToF::update() {
  if (!tofSensor.dataReady()) return false;        // nothing new: return at once
  const uint32_t t = shared::nowUs();
  tofSensor.read(false);                           // non-blocking: the data is already there
  if (tofSensor.timeoutOccurred()) timeouts_++;

  const VL53L1X::RangingData& d = tofSensor.ranging_data;
  last_ = make(t, d.range_mm, (uint8_t)d.range_status, d.peak_signal_count_rate_MCPS,
               d.ambient_count_rate_MCPS);
  if (last_.valid) {
    lastValid_ = last_;
    haveValid_ = true;
  }
  return true;
}

ToFReading ToF::make(uint32_t readyUs, uint16_t rangeMm, uint8_t status, float signalMcps,
                     float ambientMcps) const {
  ToFReading r;
  r.tUs         = readyUs - cfg_.budgetUs / 2;     // the middle of the integration time
  r.rangeM      = rangeMm * 0.001f;
  r.status      = status;
  r.signalMcps  = signalMcps;
  r.ambientMcps = ambientMcps;
  r.valid = status == 0 && r.rangeM >= cfg_.minRangeM && r.rangeM <= cfg_.maxRangeM && signalMcps >= cfg_.minSignal;
  return r;
}

uint32_t ToF::validAgeUs(uint32_t nowUs) const {
  return haveValid_ ? nowUs - lastValid_.tUs : 0xFFFFFFFFu;
}

float ToF::height(float rangeM, float roll, float pitch) {
  return rangeM * cosf(roll) * cosf(pitch);
}

}  // namespace sensors
