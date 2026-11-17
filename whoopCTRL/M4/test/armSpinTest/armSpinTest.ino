// Motor spin with the Betaflight class, M4 core (Tools > Target core: M4 Co-processor, Flash split "1MB M7 + 1MB M4").
// Needs M7/m7Runner on the M7 (starts the M4, passes log lines to the PC: python M4/test/armTest/fc_log.py).
// Wiring: Nicla TX (J2 pin 3) -> FC RX, Nicla RX (J2 pin 4) -> FC TX, GND - GND (the class opens USART1 itself).
// Betaflight: Ports tab, MSP on that UART, 115200. Close Configurator. PROPS OFF.
//
// FC answers -> TEST_MOTOR at TEST_SPEED for 2 s, stop 2 s, again. Disarmed motor test: no arming.
// One log line per second.

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#define TEST_MOTOR -1    // 0..3 = Betaflight motor 1..4, -1 = all four
#define TEST_SPEED 0.10f  // 0..1 (0.10 = 1100 us)

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
  fc.begin();
}

void loop() {
  static unsigned long logMs = 0;

  fc.update(shared::nowUs());
  const bool on = fc.linkOk() && (millis() / 2000) % 2 == 0;     // 2 s on, 2 s off
  for (int i = 0; i < 4; i++)
    if (TEST_MOTOR < 0 || TEST_MOTOR == i) fc.setMotor(i, on ? TEST_SPEED : 0.0f);

  if (millis() - logMs < 1000) return;
  logMs = millis();
  char line[160];
  snprintf(line, sizeof(line), "SPIN: %s  motors %s  battery %.2f V  errors %lu",
           fc.linkOk() ? "FC answers" : "FC silent (wiring? MSP on that UART?)", on ? "ON" : "off",
           fc.batteryV(), (unsigned long)fc.errors());
  m7.log(line);
}
