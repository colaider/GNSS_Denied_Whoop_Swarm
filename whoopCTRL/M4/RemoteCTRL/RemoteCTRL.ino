// Remote control, M4 side: PC commands (via the M7) -> Betaflight class -> FC.
// Tools > Target core: M4 Co-processor, Flash split "1MB M7 + 1MB M4". M7/RemoteCTRL on the M7.
// Wiring: Nicla TX (J2 pin 3) -> FC RX, Nicla RX (J2 pin 4) -> FC TX, GND - GND.
// Betaflight: `feature RX_MSP` (and -RX_SERIAL), MSP on that UART at 115200. Close Configurator.
//
// PKT_REMOTE (msg::RemoteCommand) -> RC_STICKS setSticks, RC_ARM arm, RC_DISARM disarm now,
// RC_MOTOR motor test (disarmed only). Mode (ACRO / ANGLE / HORIZON) from every command.
// Modes tab is written once when the FC first answers (ARM on AUX1, modes on AUX2, saved).
// Back to the PC: PKT_REMOTE_STATE at 10 Hz, and a log line when arming changes.
// The library's own safety stays on: armed and no commands for commandTimeoutMs -> level, throttle
// down, disarm (WiFi lost = it comes down).

#ifndef CORE_CM4
#error "Set Tools > Target core: M4 Co-processor"
#endif

#include <RPC.h>
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M4/DrCTRL/betaflight.cpp"

comm::CoreLink      core;
comm::Communication m7;                // to the PC through the M7 (never Serial on the M4)
drone::BetaflightConfig remoteConfig() {
  drone::BetaflightConfig c;
  c.commandTimeoutMs = 500;            // WiFi hiccups shorter than this do not start a descent
  c.channels = 18;                     // all of Betaflight's RC channels: the ones not sent become 0 = invalid
  c.armLowMs = 500;                    // switch low this long first
  c.armTimeoutMs = 4000;               // then high: Betaflight may hold arming ~1.2 s (DShot beacon guard)
  return c;
}
drone::Betaflight   fc(remoteConfig());
comm::Packet        pkt;

uint16_t lastSeq = 0;

void say(const char* fmt, unsigned long a = 0, unsigned long b = 0) {
  char line[160];
  snprintf(line, sizeof(line), fmt, a, b);
  m7.log(line);
}

void handle(const msg::RemoteCommand& c) {
  lastSeq = c.seq;
  if (c.mode <= (uint8_t)drone::FlightMode::HORIZON) fc.setMode((drone::FlightMode)c.mode);
  switch (c.action) {
    case msg::RC_DISARM:
      fc.disarm();
      break;
    case msg::RC_ARM:                  // sent repeatedly: start only once
      if (!fc.armed() && !fc.arming()) fc.arm();
      break;
    case msg::RC_MOTOR:
      for (int i = 0; i < 4; i++)
        fc.setMotor(i, (c.motor == 255 || c.motor == i) ? c.throttle : 0.0f);
      break;
    default:                           // RC_STICKS
      fc.setSticks(c.roll, c.pitch, c.yaw, c.throttle);
  }
}

void sendState() {
  msg::RemoteState s;
  s.tUs = shared::nowUs();
  s.cmdSeq = lastSeq;
  s.fcLink = fc.linkOk();
  s.armed = fc.armed() ? 2 : (fc.arming() ? 1 : 0);
  s.mode = (uint8_t)fc.mode();
  s.blockerCount = fc.armBlockerCount();
  s.blockers = fc.armBlockers();
  s.battery = fc.batteryV();
  s.roll = fc.attitude().rollDeg;
  s.pitch = fc.attitude().pitchDeg;
  s.yaw = fc.attitude().yawDeg;
  for (int i = 0; i < 8; i++) s.rc[i] = fc.rcSent()[i];
  m7.send(msg::PKT_REMOTE_STATE, &s, sizeof(s));
}

void setup() {
  comm::addLink(core);
  m7.begin("core");
  fc.begin();                          // USART1, J2 pins 3 / 4
  fc.setMode(drone::FlightMode::ANGLE);
  m7.log("REMOTE M4: started");
}

void loop() {
  static bool modesSet = false, wasArmed = false, wasArming = false, hadLink = false;
  static unsigned long stateMs = 0;

  fc.update(shared::nowUs());
  while (m7.receive(pkt))
    if (pkt.type == msg::PKT_REMOTE && pkt.len == sizeof(msg::RemoteCommand)) {
      msg::RemoteCommand c;
      memcpy(&c, pkt.data, sizeof(c));
      handle(c);
    }

  if (fc.linkOk() != hadLink) {
    hadLink = fc.linkOk();
    m7.log(hadLink ? "REMOTE M4: FC answers" : "REMOTE M4: FC silent (wiring? MSP on that UART?)");
  }
  if (fc.linkReady() && !modesSet && !fc.armed()) {
    modesSet = fc.setupModes(true);
    if (modesSet) m7.log("REMOTE M4: Modes tab written (ARM on AUX1, ANGLE / HORIZON on AUX2), saved");
  }
  if (fc.armed() != wasArmed) {
    wasArmed = fc.armed();
    m7.log(wasArmed ? "REMOTE M4: ARMED" : "REMOTE M4: disarmed");
  }
  if (wasArming && !fc.arming() && !fc.armed()) say("REMOTE M4: arming refused, blockers 0x%lx", fc.armBlockers());
  wasArming = fc.arming();

  if (millis() - stateMs >= 100) {     // 10 Hz
    stateMs = millis();
    sendState();
  }
}
