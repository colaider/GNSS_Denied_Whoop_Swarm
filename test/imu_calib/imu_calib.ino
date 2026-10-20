// Nicla Vision IMU calibration streamer (USB serial, no IMU library).
// Reads the LSM6DSOX directly over its internal SPI bus and prints every sample:
//   t_us,ax,ay,az,gx,gy,gz,temp      (g, dps, degC)
// Use with imu_calib.py (close Serial Monitor first, the script needs the port).
//
// Calibrate with the SAME data rate and ranges you will fly with:
// bias and scale change slightly between ranges.

#include <SPI.h>

// ---- IMU settings ----
// CTRL1_XL: ODR 416 Hz (0x6_) | FS +-4 g      (bits 3:2 = 10) -> 0x68
// CTRL2_G : ODR 416 Hz (0x6_) | FS +-2000 dps (bits 3:2 = 11) -> 0x6C
const uint8_t CTRL1_XL_VAL = 0x68;
const uint8_t CTRL2_G_VAL  = 0x6C;
const float ACC_LSB  = 0.122e-3f;     // g per count at +-4 g
const float GYRO_LSB = 0.070f;        // dps per count at +-2000 dps

const int CS = PIN_SPI_SS1;           // IMU chip select (internal SPI1)
SPISettings imuSpi(10000000, MSBFIRST, SPI_MODE0);

void wr(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(imuSpi); digitalWrite(CS, LOW);
  SPI1.transfer(reg); SPI1.transfer(val);
  digitalWrite(CS, HIGH); SPI1.endTransaction();
}

void rd(uint8_t reg, uint8_t* buf, int n) {
  SPI1.beginTransaction(imuSpi); digitalWrite(CS, LOW);
  SPI1.transfer(0x80 | reg);                      // bit 7 = read
  for (int i = 0; i < n; i++) buf[i] = SPI1.transfer(0);
  digitalWrite(CS, HIGH); SPI1.endTransaction();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
  SPI1.begin();

  wr(0x12, 0x01);                     // CTRL3_C: software reset
  delay(20);
  uint8_t id; rd(0x0F, &id, 1);       // WHO_AM_I, expect 0x6C
  if (id != 0x6C) { Serial.print("# IMU not found, WHO_AM_I=0x"); Serial.println(id, HEX); }

  wr(0x12, 0x44);                     // CTRL3_C: block data update + address auto-increment
  wr(0x10, CTRL1_XL_VAL);             // accel rate + range
  wr(0x11, CTRL2_G_VAL);              // gyro rate + range
}

void loop() {
  uint8_t st; rd(0x1E, &st, 1);       // STATUS_REG
  if (!(st & 0x02)) return;           // bit 1 = new gyro sample (accel runs at the same rate)

  int16_t d[7];                       // temp, gx gy gz, ax ay az (0x20..0x2D, one burst)
  rd(0x20, (uint8_t*)d, 14);
  uint32_t t = micros();

  Serial.print(t); Serial.print(',');
  Serial.print(d[4] * ACC_LSB, 5);  Serial.print(',');
  Serial.print(d[5] * ACC_LSB, 5);  Serial.print(',');
  Serial.print(d[6] * ACC_LSB, 5);  Serial.print(',');
  Serial.print(d[1] * GYRO_LSB, 3); Serial.print(',');
  Serial.print(d[2] * GYRO_LSB, 3); Serial.print(',');
  Serial.print(d[3] * GYRO_LSB, 3); Serial.print(',');
  Serial.println(25.0f + d[0] / 256.0f, 2);   // 256 counts/degC, 0 = 25 degC
}
