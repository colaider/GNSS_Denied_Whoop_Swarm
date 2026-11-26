// Prints this Nicla's unique ID (the STM32H747 UID, 0x1FF1E800) once a second on USB serial.
// Same 24-character hex as the flight firmware's boot log and the calibration folders:
//   whoopCTRL/drones/<name>_<uid>/   (e.g. drone1_0025002e3135510e34383436)
// Upload, open Serial Monitor (115200).

void setup() { Serial.begin(115200); }

void loop() {
  const volatile uint32_t* p = (const volatile uint32_t*)0x1FF1E800UL;
  char s[32];
  snprintf(s, sizeof(s), "UID %08lx%08lx%08lx", (unsigned long)p[0], (unsigned long)p[1], (unsigned long)p[2]);
  Serial.println(s);
  delay(1000);
}
