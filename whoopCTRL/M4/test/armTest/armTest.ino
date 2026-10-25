// M4 arm test: talks to the Betaflight flight controller over Serial1 (MSP) and arms it.
//
//   1. waits for the FC to answer: logs who it is (variant, version), battery, modes it has
//   2. (WRITE_MODES) puts ARM on AUX1 1700-2100 + ANGLE / HORIZON on AUX2 and saves
//   3. counts down COUNTDOWN_S, logging what would block arming right now
//   4. arms (arm switch low with throttle low, then high), holds ARMED_HOLD_S at zero
//      throttle (the motors spin at idle!), logging attitude + battery every second
//   5. disarms. If arming was refused it logs Betaflight's reasons and tries again
//      (MAX_TRIES). Reset the board to run it again.
//
// !!! PROPS OFF. First run it without the battery (USB power only): arming works without the
// ESCs too (unless bidirectional DShot blocks it: DSHOT_TELEM / RPM_FILTER in the log), and
// nothing can spin. Then with the battery, still props off.
//
// Betaflight setup (Configurator, then close it: while it is connected arming is blocked by MSP):
//   CLI:  feature -RX_SERIAL   feature RX_MSP   save      (the M4 is the receiver; drone::Link::RX_MSP)
//   Ports tab: the UART wired to the Nicla -> "Configuration/MSP" on, 115200.
// Wiring: Nicla TX -> FC RX of that UART, Nicla RX -> FC TX, GND - GND.
//
// Runs with M7/m7Runner on the M7 (it starts this core and passes our log lines to the PC).
// Logs on the PC: python M4/test/armTest/fc_log.py   (close Serial Monitor first)
// Upload: Tools > Board: Arduino Nicla Vision, Target core: M4 Co-processor, Flash split
// "1MB M7 + 1MB M4".

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#include <RPC.h>       // listed here so the Arduino IDE finds the library

// The IDE only compiles this sketch folder, so the project code is included by absolute path.
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/DrCTRL/betaflight.cpp"

const bool     WRITE_MODES  = true;   // ARM must be on AUX1 for arm() to work; false if set in the Modes tab
const uint32_t COUNTDOWN_S  = 5;
const uint32_t ARMED_HOLD_S = 5;
const int      MAX_TRIES    = 3;
const uint32_t FC_BAUD      = 115200;

comm::CoreLink      core;
comm::Communication m7;                // to the M7 and through it to the PC (log lines)
drone::Betaflight   fc;                // default config: Link::RX_MSP, ARM on AUX1, mode on AUX2

void logLine(const char* fmt, long a = 0, long b = 0, long c = 0, long d = 0, long e = 0) {
  char line[160] = "ARM TEST: ";
  snprintf(line + 10, sizeof(line) - 10, fmt, a, b, c, d, e);
  m7.log(line);
}

static void toFc(const uint8_t* d, int n, void*) { Serial1.write(d, n); }

// Betaflight's reasons it will not arm, as one line
void logBlockers(const char* when) {
  char names[120] = "";
  const uint32_t b = fc.armBlockers();
  for (int i = 0; i < 32; i++) {
    if (!(b & (1UL << i))) continue;
    if (names[0]) strncat(names, " ", sizeof(names) - strlen(names) - 1);
    strncat(names, drone::Betaflight::blockerName(i, fc.armBlockerCount()), sizeof(names) - strlen(names) - 1);
  }
  char line[160];
  snprintf(line, sizeof(line), "ARM TEST: %s: arming blocked by: %s", when, b ? names : "nothing");
  m7.log(line);
}

void logState() {
  const drone::FcAttitude a = fc.attitude();
  char line[160];
  const long v100 = (long)(fc.batteryV() * 100.0f + 0.5f);
  snprintf(line, sizeof(line), "ARM TEST: %s, roll %.1f pitch %.1f yaw %.0f deg, battery %ld.%02ld V, link %s",
           fc.armed() ? "ARMED" : "disarmed", a.rollDeg, a.pitchDeg, a.yawDeg, v100 / 100, v100 % 100,
           fc.linkOk() ? "ok" : "LOST");
  m7.log(line);
}

void setup() {
  comm::addLink(core);
  m7.begin("core");                    // connects to the M7 (it started us)
  Serial1.begin(FC_BAUD);
  fc.begin(toFc, nullptr);
  fc.setMode(drone::FlightMode::ANGLE);
  logLine("starting: waiting for the flight controller on Serial1 at %ld baud. PROPS OFF.", (long)FC_BAUD);
}

void loop() {
  enum Stage { WAIT_LINK, COUNTDOWN, ARMING, ARMED_HOLD, DONE };
  static Stage    stage = WAIT_LINK;
  static uint32_t stageUs = 0, lastLogUs = 0;
  static int      tries = 0;
  static bool     wasArmed = false;

  while (Serial1.available()) fc.receive((uint8_t)Serial1.read());
  const uint32_t now = shared::nowUs();
  fc.update(now);
  if (!stageUs) stageUs = lastLogUs = now;
  const uint32_t inStage = now - stageUs;
  const bool everySecond = now - lastLogUs >= 1000000UL;
  if (everySecond) lastLogUs = now;

  switch (stage) {
    case WAIT_LINK:
      if (fc.linkReady()) {
        const uint8_t* v = fc.fcVersion();
        char line[160];
        snprintf(line, sizeof(line), "ARM TEST: FC answered: %s %u.%u.%u, MSP API %u.%u", fc.variant(), v[0], v[1], v[2],
                 fc.apiMajor(), fc.apiMinor());
        m7.log(line);
        snprintf(line, sizeof(line), "ARM TEST: modes: ANGLE %s HORIZON %s ALTHOLD %s POSHOLD %s",
                 fc.modeAvailable(drone::Box::ANGLE) ? "yes" : "no",
                 fc.modeAvailable(drone::Box::HORIZON) ? "yes" : "no", fc.modeAvailable(drone::Box::ALTHOLD) ? "yes" : "no",
                 fc.modeAvailable(drone::Box::POSHOLD) ? "yes" : "no");
        m7.log(line);
        if (WRITE_MODES) {
          fc.setupModes(true);
          logLine("modes written and saved: ARM = AUX1 1700-2100, ANGLE = AUX2 1225-1375, HORIZON = AUX2 1425-1575");
        }
        stage = COUNTDOWN;
        stageUs = now;
      } else if (everySecond && inStage > 2000000UL) {
        logLine("no answer from the FC yet (%ld s): wiring TX/RX crossed? Ports tab MSP on that UART, %ld baud?",
                (long)(inStage / 1000000UL), (long)FC_BAUD);
      }
      break;

    case COUNTDOWN:
      fc.setAttitude(0, 0, 0, 0);
      if (everySecond) {
        logLine("arming in %ld s - PROPS OFF", (long)(COUNTDOWN_S - inStage / 1000000UL));
        logBlockers("now");
        logState();
      }
      if (inStage >= COUNTDOWN_S * 1000000UL) {
        tries++;
        logLine("arming, try %ld of %ld", tries, MAX_TRIES);
        fc.arm();
        stage = ARMING;
        stageUs = now;
      }
      break;

    case ARMING:
      fc.setAttitude(0, 0, 0, 0);      // throttle 0: the arm sequence keeps it low anyway
      if (fc.armed()) {
        logLine("ARMED after %ld ms. Holding %ld s at zero throttle (motors idle).", (long)(inStage / 1000), (long)ARMED_HOLD_S);
        stage = ARMED_HOLD;
        stageUs = now;
        wasArmed = true;
      } else if (!fc.arming()) {        // the sequence gave up
        logBlockers("refused");
        if (tries < MAX_TRIES) {
          stage = COUNTDOWN;            // fix it during the countdown (level the board, close Configurator...)
        } else {
          logLine("gave up after %ld tries", tries);
          stage = DONE;
        }
        stageUs = now;
      }
      break;

    case ARMED_HOLD:
      fc.setAttitude(0, 0, 0, 0);      // keep commanding, or Betaflight-side timeout logic descends + disarms
      if (everySecond) logState();
      if (!fc.armed() && wasArmed) {
        logBlockers("disarmed by the FC");
        stage = DONE;
        stageUs = now;
      } else if (inStage >= ARMED_HOLD_S * 1000000UL) {
        fc.disarm();
        logLine("disarmed. Test done - reset the board to run it again.");
        stage = DONE;
        stageUs = now;
      }
      break;

    case DONE:
      if (everySecond && inStage < 3000000UL) logState();   // shows the FC really disarmed
      break;
  }
}
