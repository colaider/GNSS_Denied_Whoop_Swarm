// Betaflight - the M4 commands the flight controller over a UART with MSP v1.
//
// Flight controller: BETAFPV F4 2-3S AIO FC V1 (target BETAFPVF405, ships with Betaflight 4.4.1;
// onboard ELRS on UART3, SBUS on UART5, a free UART for us). Sources: Betaflight
// src/main/msp/msp_protocol.h, msp.c, msp_box.c, fc/rc_modes.h, fc/runtime_config.h and the
// "MSP Companion Computer Control" guide (betaflight.com).
//
// What it does:
//   RC frames    MSP_SET_RAW_RC (200) at rcRateHz, always (Betaflight needs a steady stream):
//                ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw (AETR, Betaflight's default map),
//                ch5 = AUX1 arm switch, ch6 = AUX2 flight mode, ch7/8 = 1000. 1000..2000 us.
//   arm()        arm switch low + throttle low for armLowMs (Betaflight must see the switch
//                off first), then the switch high; armed() reports what the FC says.
//   setMode()    AUX2 position: ACRO 1000 (no mode on), ANGLE 1300, HORIZON 1500, ALTHOLD 1700,
//                POSHOLD 1900. setupModes() writes those ranges into the FC (or set them in the
//                Modes tab by hand). ALTHOLD / POSHOLD only exist on firmware + sensors that have
//                them (not on 4.4.1, not without a baro): modeAvailable() says.
//   setAttitude  roll / pitch angle (ANGLE / HORIZON), yaw rate, throttle -> stick positions.
//   setRates     roll / pitch / yaw rate (ACRO), throttle -> stick positions.
//   setMotor()   motor test, MSP_SET_MOTOR (214): DISARMED ONLY, PROPS OFF.
//   telemetry    MSP_STATUS (armed, modes, arming blockers), MSP_ATTITUDE, MSP_ANALOG (battery).
//
// Two ways to wire it in Betaflight (the guide's two configurations):
//   Link::RX_MSP        the M4 is the only receiver: `feature RX_MSP` (and `feature -RX_SERIAL`).
//                       It arms and flies. If the M4 stops sending, the FC fails safe. The guide
//                       calls this bench / props-off only until the software is trusted.
//   Link::MSP_OVERRIDE  the ELRS radio stays the receiver and owns ARM and an "MSP OVERRIDE"
//                       switch (Modes tab, receiver channels); `set msp_override_channels_mask
//                       = 47` gives the M4 ch1-4 + ch6 (mode) while that switch is on. arm()
//                       cannot arm here (the pilot does), and that is the point: a human kill.
// Both: Ports tab -> the UART wired to the Nicla: "Configuration/MSP" on, 115200.
// Wiring: Nicla TX -> FC RX of that UART, Nicla RX -> FC TX, GND - GND (both 3.3 V logic).
//
// Stick -> angle / rate: Betaflight maps full stick to `level_angle_limit` (4.4, default 55 deg;
// 4.5+ `angle_limit`, 60) in ANGLE mode, and to the rates curve in ACRO. maxAngleDeg / maxRateDps
// must match, and the curve must be linear (Actual rates, center sensitivity = max rate, expo 0),
// or the angles / rates sent are not the ones flown. Throttle: linear with thr_mid / thr_expo off.
//
//   drone::Betaflight fc;                               // global
//   fc.begin(writeToUart, nullptr);                     // how bytes go out
//   loop: while (Serial1.available()) fc.receive(Serial1.read());
//         fc.update(shared::nowUs());                   // RC frames, requests, timeouts
//   fc.setMode(drone::FlightMode::ANGLE);  fc.arm();
//   fc.setAttitude(2.0f, -1.0f, 0.0f, 0.35f);           // every control step (< commandTimeoutMs apart)
//
// No heap memory, nothing to free. Portable C++ (tested on the PC).

#pragma once

#include <stdint.h>

namespace drone {

// MSP v1 command codes (msp_protocol.h)
namespace msp {
enum : uint8_t {
  API_VERSION = 1, FC_VARIANT = 2, FC_VERSION = 3, MODE_RANGES = 34, SET_MODE_RANGE = 35,
  STATUS = 101, MOTOR = 104, RC = 105, ATTITUDE = 108, ANALOG = 110, BOXIDS = 119,
  SET_RAW_RC = 200, SET_MOTOR = 214, EEPROM_WRITE = 250
};
}  // namespace msp

// Betaflight modes by permanent id (msp_box.c): what MSP_BOXIDS lists, MSP_SET_MODE_RANGE takes
enum class Box : uint8_t {
  ARM = 0, ANGLE = 1, HORIZON = 2, ALTHOLD = 3, POSHOLD = 11, BEEPER = 13, CRASHFLIP = 35, MSP_OVERRIDE = 50
};

// Flight modes setMode() selects with AUX2. ACRO = no self-level mode on (rates only). A
// "hover" of our own = ANGLE + throttle from the M4's height controller (ToF); ALTHOLD /
// POSHOLD are Betaflight's (2025.12+, need a baro / GPS).
enum class FlightMode : uint8_t { ACRO, ANGLE, HORIZON, ALTHOLD, POSHOLD };

enum class Link : uint8_t { RX_MSP, MSP_OVERRIDE };

struct BetaflightConfig {
  Link     link         = Link::RX_MSP;
  uint8_t  channels     = 8;       // per RC frame: AETR + AUX1..4 (Betaflight takes up to 18)
  uint8_t  armAux       = 0;       // AUX index of the arm switch (0 = AUX1 = ch5); RX_MSP only
  uint8_t  modeAux      = 1;       // AUX index of the mode switch (1 = AUX2 = ch6)
  uint16_t rcRateHz     = 50;      // RC frames per second (the guide: 25-50 Hz, steady)
  uint16_t statusRateHz = 10;      // each of status / attitude / battery requested this often
  float    maxAngleDeg  = 55.0f;   // full stick in ANGLE / HORIZON (Betaflight 4.4 level_angle_limit)
  float    maxRateDps[3] = {670.0f, 670.0f, 670.0f};   // full stick in ACRO: roll, pitch, yaw (Actual rates, linear)
  uint32_t commandTimeoutMs = 300; // armed and no setAttitude / setRates / setSticks this long:
  float    descendPerS  = 0.25f;   // level, yaw 0, throttle falls this much per second; at 0: disarm
  uint32_t armLowMs     = 300;     // arm(): switch low + throttle low this long first
  uint32_t armTimeoutMs = 1500;    // ...then switch high; not armed by then: give up (armBlockers() says why)
  uint8_t  motorCount   = 4;
  uint32_t motorTestTimeoutMs = 1000;   // setMotor() not called this long: all motors stop
  uint8_t  modeSlotBase = 0;       // setupModes(): first Modes-tab slot it overwrites (uses up to 5)
};

struct FcAttitude {
  float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;   // Betaflight's own estimate
};

class Betaflight {
 public:
  typedef void (*WriteFn)(const uint8_t* data, int n, void* ctx);

  explicit Betaflight(const BetaflightConfig& cfg = BetaflightConfig());

  // How bytes reach the FC UART. Asks the FC who it is (API / variant / version / modes).
  void begin(WriteFn write, void* ctx);

  // Every byte read from the FC UART.
  void receive(uint8_t b);

  // Every loop: RC frames at rcRateHz, status requests, arm sequence, command / motor timeouts.
  void update(uint32_t nowUs);

  // --- arming ---
  bool arm();                      // starts the arm sequence (RX_MSP). False: not now (see link / armed / motor test)
  void disarm();                   // immediately (next RC frame goes out now)
  bool armed() const { return armed_; }
  bool arming() const { return armStage_ != 0; }
  uint32_t armBlockers() const { return armBlock_; }      // Betaflight's arming disable flags (bits)
  uint8_t  armBlockerCount() const { return armBlockCount_; }
  // Name of a blocker bit: Betaflight's names, the last bit (count - 1) is always ARM_SWITCH
  // ("switch was on at boot / not seen off"). The middle ones moved between versions a little.
  static const char* blockerName(int bit, int count);

  // --- modes ---
  void       setMode(FlightMode m) { mode_ = m; }
  FlightMode mode() const { return mode_; }
  bool       modeActive(Box b) const;      // the FC reports it on (needs the BOXIDS list: linkReady())
  bool       modeAvailable(Box b) const;   // the FC has this mode at all
  // Writes the Modes tab: ARM on AUX1 1700-2100 (RX_MSP only), ANGLE / HORIZON / ALTHOLD /
  // POSHOLD on AUX2 around 1300 / 1500 / 1700 / 1900 (+-75). Slots modeSlotBase.. are
  // overwritten. save = also EEPROM write. Disarmed only; needs linkReady() (to skip modes the
  // FC does not have). False if not possible now.
  bool setupModes(bool save);

  // --- motion (call every control step; older than commandTimeoutMs while armed = descend) ---
  void setAttitude(float rollDeg, float pitchDeg, float yawRateDps, float throttle);   // ANGLE / HORIZON
  void setRates(float rollDps, float pitchDps, float yawDps, float throttle);          // ACRO
  void setSticks(float roll, float pitch, float yaw, float throttle);   // raw: -1..1, throttle 0..1

  // --- motor test: disarmed, props off. index 0..motorCount-1 (Betaflight motor 1 = index 0;
  //     quad X: 1 rear right, 2 front right, 3 rear left, 4 front left). value 0..1 ---
  bool setMotor(int index, float value);
  void stopMotors();

  // --- what the FC reports ---
  bool       linkOk() const { return linkOk_; }            // a reply within the last second
  bool       linkReady() const { return haveBoxes_; }      // its mode list arrived
  FcAttitude attitude() const { return att_; }
  float      batteryV() const { return vbat_; }
  const char* variant() const { return variant_; }        // "BTFL"
  uint8_t    apiMajor() const { return api_[1]; }
  uint8_t    apiMinor() const { return api_[2]; }
  const uint8_t* fcVersion() const { return ver_; }       // major, minor, patch
  uint32_t   errors() const { return errors_; }           // bad checksums + error replies
  const uint16_t* rcSent() const { return rc_; }          // the last RC frame, us

  // MSP v1 frame "$M<" size cmd payload xor-checksum into out (6 + n bytes). Public for testing.
  static int encode(uint8_t cmd, const uint8_t* payload, int n, uint8_t* out);

 private:
  void send(uint8_t cmd, const uint8_t* payload, int n);
  void sendRc();
  void sendMotors();
  void handle(uint8_t cmd, const uint8_t* d, int n);
  void command(float r, float p, float y, float t);       // stores sticks, marks the time
  static uint16_t us(float v);                             // -1..1 -> 1000..2000
  static uint16_t usThrottle(float v);                     // 0..1 -> 1000..2000

  BetaflightConfig cfg_;
  WriteFn  write_ = nullptr;
  void*    ctx_   = nullptr;
  uint32_t now_   = 0;

  // commands
  float      stick_[4] = {0, 0, 0, 0};                     // roll, pitch, yaw -1..1; throttle 0..1
  uint32_t   cmdUs_    = 0;
  bool       haveCmd_  = false;
  FlightMode mode_     = FlightMode::ANGLE;
  bool       armSwitch_ = false;
  int        armStage_ = 0;                                // 0 idle, 1 switch low, 2 switch high (waiting)
  uint32_t   armStageUs_ = 0;
  uint16_t   motor_[8];
  bool       motorTest_ = false;
  uint32_t   motorUs_   = 0;
  uint16_t   rc_[18];
  uint32_t   rcUs_ = 0, reqUs_ = 0, replyUs_ = 0;
  int        reqTurn_ = 0;

  // from the FC
  bool       armed_ = false, linkOk_ = false, haveBoxes_ = false;
  uint32_t   armBlock_ = 0;
  uint8_t    armBlockCount_ = 0;
  uint8_t    boxIds_[64];
  int        nBoxes_ = 0;
  uint8_t    boxFlags_[16];                                // active-mode bits, in BOXIDS order
  FcAttitude att_;
  float      vbat_ = 0.0f;
  char       variant_[5] = {0};
  uint8_t    api_[3] = {0, 0, 0}, ver_[3] = {0, 0, 0};
  uint32_t   errors_ = 0;

  // receive state
  int     rxState_ = 0;
  bool    rxError_ = false;
  uint8_t rxSize_ = 0, rxCmd_ = 0, rxSum_ = 0, rxN_ = 0;
  uint8_t rxBuf_[128];
};

}  // namespace drone
