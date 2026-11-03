// Simplest arm test, M4 core (Tools > Target core: M4 Co-processor, Flash split "1MB M7 + 1MB M4").
// Needs M7/m7Runner on the M7 (starts the M4, passes log lines to the PC: python fc_log.py).
// Wiring: Nicla TX (J2 pin 3) -> FC RX, Nicla RX (J2 pin 4) -> FC TX, GND - GND.
// Betaflight: CLI `feature -RX_SERIAL`, `feature RX_MSP`, `save`; Ports tab: MSP on that UART, 115200.
// Close Betaflight Configurator before running (while connected it blocks arming).
//
// FC answers -> mode ranges written (ARM on AUX1) -> 5 s wait -> arm -> 5 s armed at zero
// throttle (MOTORS SPIN AT IDLE: PROPS OFF) -> disarm. One log line per second.

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#include <RPC.h>
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/DrCTRL/betaflight.cpp"

comm::CoreLink      core;
comm::Communication m7;                // log lines to the PC through the M7 (never Serial on the M4)
drone::Betaflight   fc;

void setup() {
  comm::addLink(core);
  m7.begin("core");
  fc.begin();                          // opens USART1 (Serial1 does not reach J2 pins 3/4 from the M4)
  fc.setMode(drone::FlightMode::ANGLE);
}

void loop() {
  static unsigned long linkMs = 0, logMs = 0;
  static bool armSent = false, disarmSent = false;

  fc.update(shared::nowUs());
  fc.setAttitude(0, 0, 0, 0);          // level, zero throttle; sent every loop so it never goes stale

  if (fc.linkReady() && !linkMs) {     // first answer: ARM on AUX1, ANGLE / HORIZON on AUX2, saved
    linkMs = millis();
    fc.setupModes(true);
  }
  const unsigned long t = linkMs ? millis() - linkMs : 0;
  if (linkMs && t > 5000 && !armSent) { fc.arm(); armSent = true; }
  if (linkMs && t > 10000 && !disarmSent) { fc.disarm(); disarmSent = true; }

  if (millis() - logMs < 1000) return;
  logMs = millis();
  char line[160];
  if (!linkMs) {
    snprintf(line, sizeof(line), "ARM: waiting for the FC (wiring? MSP on that UART?)");
  } else {
    char why[100] = "";                // Betaflight's reasons it would not arm now
    for (int i = 0; i < 32; i++)
      if (fc.armBlockers() & (1UL << i)) {
        strncat(why, drone::Betaflight::blockerName(i, fc.armBlockerCount()), sizeof(why) - strlen(why) - 1);
        strncat(why, " ", sizeof(why) - strlen(why) - 1);
      }
    snprintf(line, sizeof(line), "ARM: %s %u.%u.%u  t %lu s  %s  battery %.2f V  blocked by: %s", fc.variant(),
             fc.fcVersion()[0], fc.fcVersion()[1], fc.fcVersion()[2], t / 1000, fc.armed() ? "ARMED" : "disarmed",
             fc.batteryV(), why[0] ? why : "nothing");
  }
  m7.log(line);
}
