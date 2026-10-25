#include "headers/betaflight.h"

namespace drone {

// 'bf' prefix: sketches include several .cpp files into one compile unit
static float bfClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint16_t bfU16(const uint8_t* d) { return (uint16_t)(d[0] | (d[1] << 8)); }

// AUX2 position of each flight mode, and the step range setupModes() gives it (+-75 us).
// Betaflight mode ranges are in 25 us steps from 900: step = (us - 900) / 25 (rc_modes.h).
static const uint16_t BF_MODE_US[5] = {1000, 1300, 1500, 1700, 1900};   // ACRO, ANGLE, HORIZON, ALTHOLD, POSHOLD
static uint8_t bfStep(int us) { return (uint8_t)((us - 900) / 25); }

Betaflight::Betaflight(const BetaflightConfig& cfg) : cfg_(cfg) {
  if (cfg_.channels < 6) cfg_.channels = 6;
  if (cfg_.channels > 18) cfg_.channels = 18;
  if (cfg_.motorCount > 8) cfg_.motorCount = 8;
  if (4 + cfg_.armAux >= cfg_.channels) cfg_.armAux = 0;    // both switches must be inside the frame
  if (4 + cfg_.modeAux >= cfg_.channels || cfg_.modeAux == cfg_.armAux) cfg_.modeAux = cfg_.armAux + 1;
  for (int i = 0; i < 8; i++) motor_[i] = 1000;
  for (int i = 0; i < 18; i++) rc_[i] = 1000;
}

void Betaflight::begin(WriteFn write, void* ctx) {
  write_ = write;
  ctx_ = ctx;
  send(msp::API_VERSION, nullptr, 0);
  send(msp::FC_VARIANT, nullptr, 0);
  send(msp::FC_VERSION, nullptr, 0);
  send(msp::BOXIDS, nullptr, 0);
}

// ------------------------------------------------------------------ frames

int Betaflight::encode(uint8_t cmd, const uint8_t* payload, int n, uint8_t* out) {
  out[0] = '$';
  out[1] = 'M';
  out[2] = '<';                                    // to the FC
  out[3] = (uint8_t)n;
  out[4] = cmd;
  uint8_t sum = (uint8_t)n ^ cmd;
  for (int i = 0; i < n; i++) {
    out[5 + i] = payload[i];
    sum ^= payload[i];
  }
  out[5 + n] = sum;
  return 6 + n;
}

void Betaflight::send(uint8_t cmd, const uint8_t* payload, int n) {
  if (!write_) return;
  uint8_t buf[6 + 64];
  if (n > 64) return;
  write_(buf, encode(cmd, payload, n, buf), ctx_);
}

uint16_t Betaflight::us(float v) { return (uint16_t)(1500.0f + 500.0f * bfClamp(v, -1.0f, 1.0f) + 0.5f); }
uint16_t Betaflight::usThrottle(float v) { return (uint16_t)(1000.0f + 1000.0f * bfClamp(v, 0.0f, 1.0f) + 0.5f); }

void Betaflight::sendRc() {
  // AETR + AUX: roll, pitch, throttle, yaw, AUX1.. (Betaflight's default channel map)
  for (int i = 0; i < cfg_.channels; i++) rc_[i] = 1000;
  rc_[0] = us(stick_[0]);
  rc_[1] = us(stick_[1]);
  rc_[2] = usThrottle(stick_[3]);
  rc_[3] = us(stick_[2]);
  // the arm channel only means something with RX_MSP (with MSP override the radio owns ARM)
  rc_[4 + cfg_.armAux]  = (cfg_.link == Link::RX_MSP && armSwitch_) ? 2000 : 1000;
  rc_[4 + cfg_.modeAux] = BF_MODE_US[(int)mode_];
  uint8_t p[36];
  for (int i = 0; i < cfg_.channels; i++) {
    p[2 * i]     = (uint8_t)(rc_[i] & 0xff);
    p[2 * i + 1] = (uint8_t)(rc_[i] >> 8);
  }
  send(msp::SET_RAW_RC, p, 2 * cfg_.channels);
  rcUs_ = now_;
}

void Betaflight::sendMotors() {
  uint8_t p[16];                                   // Betaflight reads its motor count, extra ignored
  for (int i = 0; i < 8; i++) {
    p[2 * i]     = (uint8_t)(motor_[i] & 0xff);
    p[2 * i + 1] = (uint8_t)(motor_[i] >> 8);
  }
  send(msp::SET_MOTOR, p, 16);
}

// ------------------------------------------------------------------ loop

void Betaflight::update(uint32_t nowUs) {
  const float dt = now_ ? (int32_t)(nowUs - now_) * 1e-6f : 0.0f;
  now_ = nowUs;

  // arm sequence: throttle low throughout; switch low first so Betaflight sees it off, then high
  if (armStage_) {
    stick_[3] = 0.0f;
    if (armStage_ == 1 && (uint32_t)(now_ - armStageUs_) >= cfg_.armLowMs * 1000UL) {
      armStage_ = 2;
      armStageUs_ = now_;
      armSwitch_ = true;
    } else if (armStage_ == 2) {
      if (armed_) {
        armStage_ = 0;
        haveCmd_ = false;                          // no command yet: descend logic keeps it at 0
        cmdUs_ = now_;
      } else if ((uint32_t)(now_ - armStageUs_) >= cfg_.armTimeoutMs * 1000UL) {
        armStage_ = 0;                             // refused: armBlockers() says why
        armSwitch_ = false;
      }
    }
  }

  // armed but the commands stopped: level, no yaw, throttle down, then disarm
  if (armed_ && !armStage_ && (uint32_t)(now_ - cmdUs_) > cfg_.commandTimeoutMs * 1000UL) {
    stick_[0] = stick_[1] = stick_[2] = 0.0f;
    stick_[3] -= cfg_.descendPerS * dt;
    if (stick_[3] <= 0.0f) {
      stick_[3] = 0.0f;
      disarm();
    }
  }

  if (motorTest_ && (uint32_t)(now_ - motorUs_) > cfg_.motorTestTimeoutMs * 1000UL) stopMotors();

  if ((uint32_t)(now_ - rcUs_) >= 1000000UL / cfg_.rcRateHz) {
    sendRc();
    if (motorTest_) sendMotors();                  // keep repeating what the test asked for
  }

  // requests: status / attitude / battery in turn; the FC's identity until it answered
  const uint32_t reqPeriod = 1000000UL / (cfg_.statusRateHz * 3u);
  if ((uint32_t)(now_ - reqUs_) >= reqPeriod) {
    reqUs_ = now_;
    const uint8_t order[3] = {msp::STATUS, msp::ATTITUDE, msp::ANALOG};
    send(order[reqTurn_], nullptr, 0);
    reqTurn_ = (reqTurn_ + 1) % 3;
    if (!haveBoxes_ && reqTurn_ == 0) {
      send(msp::API_VERSION, nullptr, 0);
      send(msp::FC_VARIANT, nullptr, 0);
      send(msp::FC_VERSION, nullptr, 0);
      send(msp::BOXIDS, nullptr, 0);
    }
  }
  linkOk_ = replyUs_ && (uint32_t)(now_ - replyUs_) < 1000000UL;
}

// ------------------------------------------------------------------ commands

bool Betaflight::arm() {
  if (cfg_.link != Link::RX_MSP) return false;     // the pilot arms with the radio
  if (armed_) return true;
  if (motorTest_) stopMotors();
  armStage_ = 1;
  armStageUs_ = now_;
  armSwitch_ = false;
  return true;
}

void Betaflight::disarm() {
  armStage_ = 0;
  armSwitch_ = false;
  stick_[3] = 0.0f;
  haveCmd_ = false;
  sendRc();                                        // now, not at the next RC period
}

void Betaflight::command(float r, float p, float y, float t) {
  stick_[0] = bfClamp(r, -1.0f, 1.0f);
  stick_[1] = bfClamp(p, -1.0f, 1.0f);
  stick_[2] = bfClamp(y, -1.0f, 1.0f);
  if (!armStage_) stick_[3] = bfClamp(t, 0.0f, 1.0f);   // arming keeps the throttle low
  cmdUs_ = now_;
  haveCmd_ = true;
}

void Betaflight::setSticks(float roll, float pitch, float yaw, float throttle) { command(roll, pitch, yaw, throttle); }

void Betaflight::setAttitude(float rollDeg, float pitchDeg, float yawRateDps, float throttle) {
  command(rollDeg / cfg_.maxAngleDeg, pitchDeg / cfg_.maxAngleDeg, yawRateDps / cfg_.maxRateDps[2], throttle);
}

void Betaflight::setRates(float rollDps, float pitchDps, float yawDps, float throttle) {
  command(rollDps / cfg_.maxRateDps[0], pitchDps / cfg_.maxRateDps[1], yawDps / cfg_.maxRateDps[2], throttle);
}

bool Betaflight::setMotor(int index, float value) {
  if (armed_ || armSwitch_ || armStage_ || index < 0 || index >= cfg_.motorCount) return false;
  motor_[index] = usThrottle(value);
  motorTest_ = true;
  motorUs_ = now_;
  sendMotors();
  return true;
}

void Betaflight::stopMotors() {
  for (int i = 0; i < 8; i++) motor_[i] = 1000;
  motorTest_ = false;
  sendMotors();
}

// ------------------------------------------------------------------ modes

bool Betaflight::modeAvailable(Box b) const {
  for (int i = 0; i < nBoxes_; i++)
    if (boxIds_[i] == (uint8_t)b) return true;
  return false;
}

bool Betaflight::modeActive(Box b) const {
  for (int i = 0; i < nBoxes_; i++)                // the status bits follow the BOXIDS order
    if (boxIds_[i] == (uint8_t)b) return (boxFlags_[i / 8] >> (i % 8)) & 1;
  return false;
}

bool Betaflight::setupModes(bool save) {
  if (armed_ || !haveBoxes_) return false;
  uint8_t slot = cfg_.modeSlotBase;
  // payload: slot, permanent id, aux index, start step, end step, logic (OR), linked (none)
  auto range = [&](Box b, uint8_t aux, int lowUs, int highUs) {
    const uint8_t p[7] = {slot++, (uint8_t)b, aux, bfStep(lowUs), bfStep(highUs), 0, 0};
    send(msp::SET_MODE_RANGE, p, 7);
  };
  if (cfg_.link == Link::RX_MSP) range(Box::ARM, cfg_.armAux, 1700, 2100);   // never on an MSP channel otherwise
  const Box boxes[4] = {Box::ANGLE, Box::HORIZON, Box::ALTHOLD, Box::POSHOLD};
  for (int k = 0; k < 4; k++)
    if (modeAvailable(boxes[k])) range(boxes[k], cfg_.modeAux, BF_MODE_US[k + 1] - 75, BF_MODE_US[k + 1] + 75);
  if (save) send(msp::EEPROM_WRITE, nullptr, 0);
  return true;
}

const char* Betaflight::blockerName(int bit, int count) {
  if (bit == count - 1) return "ARM_SWITCH";     // always the last one
  static const char* const names[] = {
      "NO_GYRO", "FAILSAFE", "RX_LOSS", "BAD_RX/NOT_DISARMED", "BOX_FAILSAFE", "RUNAWAY_TAKEOFF", "CRASH",
      "THROTTLE", "ANGLE", "BOOT_GRACE", "NO_PREARM", "LOAD", "CALIBRATING", "CLI", "CMS_MENU", "BST", "MSP",
      "PARALYZE", "GPS", "RESCUE_SWITCH", "RPM_FILTER/DSHOT_TELEM", "REBOOT_REQUIRED", "DSHOT_BITBANG",
      "ACC_CALIBRATION", "MOTOR_PROTOCOL", "CRASHFLIP", "ALTHOLD", "POSHOLD", "AUTOPILOT"};
  return (bit >= 0 && bit < (int)(sizeof(names) / sizeof(names[0]))) ? names[bit] : "?";
}

// ------------------------------------------------------------------ replies

void Betaflight::receive(uint8_t b) {
  switch (rxState_) {
    case 0: rxState_ = (b == '$') ? 1 : 0; break;
    case 1: rxState_ = (b == 'M') ? 2 : 0; break;
    case 2:                                        // '>' reply, '!' error reply
      if (b == '>' || b == '!') {
        rxError_ = (b == '!');
        rxState_ = 3;
      } else {
        rxState_ = 0;
      }
      break;
    case 3:
      rxSize_ = b;
      rxSum_ = b;
      rxN_ = 0;
      rxState_ = (rxSize_ <= sizeof(rxBuf_)) ? 4 : 0;
      break;
    case 4:
      rxCmd_ = b;
      rxSum_ ^= b;
      rxState_ = rxSize_ ? 5 : 6;
      break;
    case 5:
      rxBuf_[rxN_++] = b;
      rxSum_ ^= b;
      if (rxN_ >= rxSize_) rxState_ = 6;
      break;
    case 6:
      if (b == rxSum_ && !rxError_) {
        replyUs_ = now_ ? now_ : 1;
        handle(rxCmd_, rxBuf_, rxSize_);
      } else {
        errors_++;
      }
      rxState_ = 0;
      break;
  }
}

void Betaflight::handle(uint8_t cmd, const uint8_t* d, int n) {
  switch (cmd) {
    case msp::STATUS: {
      // cycle u16, i2c errors u16, sensors u16, mode bits [6..9], profile u8, load u16,
      // u16, count u8 [15] + that many more mode bytes, arming blockers: count u8, flags u32
      if (n < 16) return;
      for (int i = 0; i < 16; i++) boxFlags_[i] = 0;
      for (int i = 0; i < 4; i++) boxFlags_[i] = d[6 + i];
      int extra = d[15];
      if (extra > 12) extra = 12;
      if (n < 16 + extra) return;
      for (int i = 0; i < extra; i++) boxFlags_[4 + i] = d[16 + i];
      armed_ = boxFlags_[0] & 1;                   // ARM is box 0: always the first mode bit
      const int a = 16 + extra;
      if (n >= a + 5) {
        armBlockCount_ = d[a];
        armBlock_ = (uint32_t)d[a + 1] | ((uint32_t)d[a + 2] << 8) | ((uint32_t)d[a + 3] << 16) | ((uint32_t)d[a + 4] << 24);
      }
      break;
    }
    case msp::ATTITUDE:                            // roll / pitch in 0.1 deg, yaw in deg
      if (n < 6) return;
      att_.rollDeg  = (int16_t)bfU16(d) * 0.1f;
      att_.pitchDeg = (int16_t)bfU16(d + 2) * 0.1f;
      att_.yawDeg   = (int16_t)bfU16(d + 4);
      break;
    case msp::ANALOG:                              // u8 0.1 V (old), u16 mAh, u16 rssi, i16 0.01 A, u16 0.01 V
      if (n >= 9) vbat_ = bfU16(d + 7) * 0.01f;
      else if (n >= 1) vbat_ = d[0] * 0.1f;
      break;
    case msp::BOXIDS:
      nBoxes_ = n < (int)sizeof(boxIds_) ? n : (int)sizeof(boxIds_);
      for (int i = 0; i < nBoxes_; i++) boxIds_[i] = d[i];
      haveBoxes_ = true;
      break;
    case msp::API_VERSION:
      if (n >= 3) for (int i = 0; i < 3; i++) api_[i] = d[i];
      break;
    case msp::FC_VARIANT:
      if (n >= 4) {
        for (int i = 0; i < 4; i++) variant_[i] = (char)d[i];
        variant_[4] = 0;
      }
      break;
    case msp::FC_VERSION:
      if (n >= 3) for (int i = 0; i < 3; i++) ver_[i] = d[i];
      break;
    default:
      break;                                       // acks of the set commands: nothing to read
  }
}

}  // namespace drone
