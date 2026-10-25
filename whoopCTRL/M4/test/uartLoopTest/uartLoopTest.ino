// UART loopback diagnostic: does the Nicla's header UART work, and from which core?
12345678910111213141516171819202122232425
    silent++;
    led(false, false, true);            // blue: the jumper / pins
  } else if (got == 0) {
    silent++;
    led(true, false, false);            // red: pins fine, UART silent
  } else {
    bad++;
    led(true, true, false);             // yellow: wrong bytes
  }
  if (now - lastLogUs >= 1000000UL) {


// Jumper: J2 pin 3 (TX, PA9) -> J2 pin 4 (RX, PA10), nothing else on those pins.
//
// Builds for either core (Tools > Target core):
//   M4  logs through the M7 (run M7/m7Runner there; it boots this core): python M4/test/armTest/fc_log.py
//   M7  logs over USB directly (nothing on the M4 needed):                 python M4/test/armTest/fc_log.py
// Same result on both = wiring / pins; works on the M7 only = an M4 problem.
//
// Steps, each logged:
//   1. pin check, no UART: PA9 driven high / low as a plain output, PA10 read back. Proves the
//      jumper sits on the right two pins (and nothing else holds them).
//   2. how PA9 / PA10 are configured after Serial1.begin(): GPIO mode and alternate function
//      (AF7 = USART1, AF3 = LPUART1 - this chip can put either on these pins).
//   3. 256 bytes at 115200 through: Serial1, USART1 on PA9/PA10, LPUART1 on PA9/PA10.
//   4. forever: the first one that worked (or Serial1) every 200 ms, a summary each second.
// LED: blue = pin check failed (jumper / pins), red = pins fine but no UART echoes,
//      yellow = bytes come back wrong, green blink = working.

#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/sharedClock.cpp"
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/communication.cpp"
#ifdef CORE_CM7
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/M7/COMM/usbLink.cpp"
comm::UsbLink link;
const char* LINK_NAME = "usb";
const char* CORE_NAME = "M7";
#else
#include <RPC.h>
#include "C:/Users/penturas/Desktop/codits/GNSS denied Whoop swarm/whoopCTRL/shared/communication/coreLink.cpp"
comm::CoreLink link;
const char* LINK_NAME = "core";
const char* CORE_NAME = "M4";
#endif

const uint32_t BAUD = 115200;

comm::Communication pc;                // log lines to the PC
arduino::UART uartUsart1(PA_9, PA_10, NC, NC);             // pin map's first choice: USART1 (AF7)
arduino::UART uartLpuart1(PA_9_ALT0, PA_10_ALT0, NC, NC);  // the alternative: LPUART1 (AF3)
arduino::UART* useUart = &Serial1;

void logLine(const char* fmt, long a = 0, long b = 0, long c = 0, long d = 0, long e = 0) {
  char line[160];
  int n = snprintf(line, sizeof(line), "UART LOOP %s: ", CORE_NAME);
  snprintf(line + n, sizeof(line) - n, fmt, a, b, c, d, e);
  pc.log(line);
}

void led(bool r, bool g, bool b) {     // the Nicla's LEDs are on when the pin is LOW
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

// GPIO mode (0 in, 1 out, 2 alternate, 3 analog) and alternate function number of PA<pin>
void logPin(const char* when, int pin) {
  const long mode = (GPIOA->MODER >> (2 * pin)) & 3;
  const long af   = (GPIOA->AFR[pin / 8] >> (4 * (pin % 8))) & 15;
  const long in   = (GPIOA->IDR >> pin) & 1;
  char line[160];
  snprintf(line, sizeof(line), "UART LOOP %s: %s PA%d: mode %ld (0 in 1 out 2 alt 3 analog), AF %ld (7 USART1, 3 LPUART1), level %ld",
           CORE_NAME, when, pin, mode, af, in);
  pc.log(line);
}

// 1) the jumper, without any UART: PA9 output, PA10 input with a pull the other way
bool pinCheck() {
  int follow = 0;
  {
    mbed::DigitalOut tx(PA_9, 0);
    mbed::DigitalIn rx(PA_10, PullUp);           // pulled up: only a real wire to a low PA9 reads 0
    for (int i = 0; i < 10; i++) {
      tx = i & 1;
      delayMicroseconds(200);
      if (rx.read() == (i & 1)) follow++;
    }
  }
  logLine("pin check: PA10 followed PA9 %ld of 10 times", follow);
  pc.log(follow == 10 ? "UART LOOP: pin check ok - the jumper connects PA9 and PA10"
                      : "UART LOOP: pin check FAILED - jumper not on J2 pins 3 and 4, or something holds the pins");
  return follow == 10;
}

// Sends n bytes, waits for them back. Returns how many came back; wrong = how many differ.
int roundTrip(arduino::UART& u, const uint8_t* out, int n, int& wrong) {
  while (u.available()) u.read();
  wrong = 0;
  int got = 0;
  for (int i = 0; i < n; i += 16) {
    const int k = (n - i < 16) ? n - i : 16;
    u.write(out + i, k);
    const uint32_t t0 = shared::nowUs();
    int have = 0;
    while (have < k && shared::nowUs() - t0 < (uint32_t)k * 10u * 1000000u / BAUD + 20000u) {
      while (u.available() && have < k) {
        if ((uint8_t)u.read() != out[i + have]) wrong++;
        have++;
      }
    }
    got += have;
    if (have == 0) break;
  }
  return got;
}

bool tryUart(const char* name, arduino::UART& u) {
  static uint8_t pattern[256];
  for (int i = 0; i < 256; i++) pattern[i] = (uint8_t)i;
  u.begin(BAUD);
  delay(20);
  logPin(name, 9);
  logPin(name, 10);
  int wrong;
  const int got = roundTrip(u, pattern, 256, wrong);
  u.end();
  char line[160];
  snprintf(line, sizeof(line), "UART LOOP %s: %s at %lu baud: sent 256, back %d, wrong %d -> %s", CORE_NAME, name,
           (unsigned long)BAUD, got, wrong, (got == 256 && wrong == 0) ? "WORKS" : (got == 0 ? "nothing back" : "bytes wrong"));
  pc.log(line);
  return got == 256 && wrong == 0;
}

bool pinsOk = false;

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  led(false, false, false);
  comm::addLink(link);
  pc.begin(LINK_NAME);
  delay(1500);                          // time to open the log on the PC

  logPin("at start", 9);
  logPin("at start", 10);
  pinsOk = pinCheck();
  const bool s1 = tryUart("Serial1", Serial1);
  const bool u1 = tryUart("USART1 PA9/PA10", uartUsart1);
  const bool l1 = tryUart("LPUART1 PA9/PA10", uartLpuart1);
  useUart = s1 ? (arduino::UART*)&Serial1 : (u1 ? &uartUsart1 : (l1 ? &uartLpuart1 : (arduino::UART*)&Serial1));
  logLine("result: Serial1 %ld, USART1 %ld, LPUART1 %ld (1 = works). Repeating with the first that works.", s1, u1, l1);
  useUart->begin(BAUD);
}

void loop() {
  static uint32_t lastUs = 0, lastLogUs = 0, rounds = 0, good = 0, silent = 0, bad = 0;
  static uint8_t counter = 0;
  const uint32_t now = shared::nowUs();
  if (now - lastUs < 200000UL) return;
  lastUs = now;

  const uint8_t msg[7] = {'$', 'M', '<', 0, 101, 101, counter++};   // MSP-like, different every round
  int wrong;
  const int got = roundTrip(*useUart, msg, 7, wrong);
  rounds++;
  if (got == 7 && wrong == 0) {
    good++;
    led(false, true, false);
    delay(30);
    led(false, false, false);
  } else if (!pinsOk) {
    silent++;
    led(false, false, true);            // blue: the jumper / pins
  } else if (got == 0) {
    silent++;
    led(true, false, false);            // red: pins fine, UART silent
  } else {
    bad++;
    led(true, true, false);             // yellow: wrong bytes
  }
  if (now - lastLogUs >= 1000000UL) {
    lastLogUs = now;
    logLine("%ld rounds: %ld ok, %ld nothing back, %ld wrong", (long)rounds, (long)good, (long)silent, (long)bad);
  }
}
