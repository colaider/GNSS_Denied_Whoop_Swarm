// Remote control, M7 side: WiFi <-> M4 forwarding only (no camera).
//   PC (remoteCTRL.py) --WiFi UDP--> M7 --core link--> M4 (M4/RemoteCTRL) --UART--> Betaflight
//   PKT_REMOTE goes to the M4; PKT_REMOTE_STATE and logs come back to the PC the same way.
//
// Upload: Tools > Board: Arduino Nicla Vision, Target core: Main Core, Flash split "1MB M7 + 1MB M4".
// WIFI_AP 1: the Nicla makes its own network: join WIFI_SSID / WIFI_PASS on the PC, the Nicla is
// 192.168.3.1. WIFI_AP 0: the Nicla joins your network; its address is logged over USB.
// USB (optional, Serial Monitor closed): python M4/test/armTest/fc_log.py shows the same logs.
// LED: red = WiFi failed, blue blink = WiFi up waiting for the PC, green blink = the PC talks to us.
// Needs the WiFi firmware once (File > Examples > STM32H747_System > WiFiFirmwareUpdater).

#ifndef CORE_CM7
#error "Set Tools > Target core: Main Core (M7)"
#endif

#include <RPC.h>
#include <WiFi.h>

#define WIFI_AP    0                   // 1: own network, 0: join WIFI_SSID
#define WIFI_SSID  "Penturas (2)"
#define WIFI_PASS  "penturas3"         // at least 8 characters
#define WIFI_PORT  4210                // UDP, the same in remoteCTRL.py

#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/wifiLink.cpp"

comm::UsbLink       usb;
comm::CoreLink      core;
comm::WifiLink      wifi(WIFI_SSID, WIFI_PASS, WIFI_AP, WIFI_PORT);
comm::Communication pc;                // USB: logs only (may be unplugged)
comm::Communication air;               // WiFi: the remote
comm::Communication m4;
comm::Packet        pkt;               // one buffer (1.4 KB), used in turn

bool     wifiOk = false;
uint32_t toM4 = 0, fromM4 = 0;

void say(const char* text) {           // to both PCs that may listen
  if (Serial) pc.log(text);
  air.log(text);
}

void setLED(bool r, bool g, bool b) {  // Nicla LEDs are on when LOW
  digitalWrite(LEDR, !r); digitalWrite(LEDG, !g); digitalWrite(LEDB, !b);
}

void setup() {
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  setLED(0, 0, 0);
  comm::addLink(usb);
  comm::addLink(core);
  comm::addLink(wifi);
  pc.begin("usb");
  pc.log("REMOTE M7: starting");

  wifiOk = air.begin("wifi");
  char line[120];
  snprintf(line, sizeof(line), wifiOk ? "REMOTE M7: WiFi %s '%s', address %s, UDP port %d"
                                      : "REMOTE M7: WiFi FAILED (%s '%s') - firmware? password >= 8?",
           WIFI_AP ? "own network" : "joined", WIFI_SSID, wifi.ip(), WIFI_PORT);
  pc.log(line);
  if (!wifiOk) setLED(1, 0, 0);

  const bool m4ok = m4.begin("core");  // starts the M4, waits up to 5 s for it
  say(m4ok ? "REMOTE M7: M4 started" : "REMOTE M7: M4 not answering (no M4 program?)");
}

void loop() {
  static unsigned long statMs = 0;

  while (air.receive(pkt))             // PC -> M4
    if (msg::forM4(pkt.type) && m4.send(pkt.type, pkt.data, pkt.len)) toM4++;
  while (m4.receive(pkt)) {            // M4 -> PC (WiFi, and USB when plugged in)
    if (!msg::fromM4(pkt.type)) continue;
    fromM4++;
    air.send(pkt.type, pkt.data, pkt.len);
    if (Serial && pkt.type == msg::PKT_LOG) pc.send(pkt.type, pkt.data, pkt.len);
  }

  const unsigned long now = millis();
  if (wifiOk) {                        // blink: green = PC connected, blue = waiting for it
    const bool on = (now / 250) % 4 == 0;
    setLED(0, on && wifi.hasPeer(), on && !wifi.hasPeer());
  }
  if (now - statMs >= 5000) {
    statMs = now;
    char line[120];
    snprintf(line, sizeof(line), "REMOTE M7: WiFi %s %s, to M4 %lu, from M4 %lu packets", wifi.ip(),
             wifi.hasPeer() ? "PC connected" : "waiting for the PC", (unsigned long)toM4, (unsigned long)fromM4);
    if (Serial) pc.log(line);
  }
}
