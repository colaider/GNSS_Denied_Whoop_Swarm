#include "headers/IMU.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "../../shared/headers/sharedClock.h"

namespace sensors {

// LSM6DSOX, the settings imu_calib.h was measured with (416 Hz, +-4 g, +-2000 dps)
static const uint8_t REG_WHO_AM_I = 0x0F, REG_CTRL1_XL = 0x10, REG_CTRL2_G = 0x11;
static const uint8_t REG_CTRL3_C = 0x12, REG_STATUS = 0x1E, REG_OUTX_L_G = 0x22;
static const uint8_t CTRL1_XL_VAL = 0x68;          // 416 Hz | +-4 g
static const uint8_t CTRL2_G_VAL  = 0x6C;          // 416 Hz | +-2000 dps
static const float   ACC_LSB      = 0.122e-3f;     // g per count
static const float   GYRO_LSB     = 0.070f;        // dps per count

static const float DEG2RAD   = 0.017453293f;
static const float GRAVITY   = 9.80665f;           // m/s^2
static const float STILL_DPS = 1.0f;               // startup: gyro noise above this = moving

static const int CS = PIN_SPI_SS1;                 // IMU chip select, internal SPI1
static SPISettings imuSpi(10000000, MSBFIRST, SPI_MODE0);

// ---------------------------------------------------------------- setup

IMU::IMU(const ImuConfig& cfg) : cfg_(cfg) {
  // start from the calibrated gyro bias (startup() measures a fresh one)
  bias_ = {calib::GYRO_BIAS_DPS[0] * calib::GYRO_SCALE[0] * DEG2RAD,
           calib::GYRO_BIAS_DPS[1] * calib::GYRO_SCALE[1] * DEG2RAD,
           calib::GYRO_BIAS_DPS[2] * calib::GYRO_SCALE[2] * DEG2RAD};
}

bool IMU::begin() {
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI1.begin();

  writeReg(REG_CTRL3_C, 0x01);                     // software reset
  delay(20);
  uint8_t id = 0;
  readRegs(REG_WHO_AM_I, &id, 1);
  if (id != 0x6C) return false;

  writeReg(REG_CTRL3_C, 0x44);                     // block data update + address auto-increment
  writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);
  writeReg(REG_CTRL2_G, CTRL2_G_VAL);
  return true;
}

bool IMU::startup(uint32_t ms) {
  // average while still: gyro -> bias, accelerometer -> which way is up
  float gs[3] = {0, 0, 0}, gq[3] = {0, 0, 0}, as[3] = {0, 0, 0};
  int n = 0;
  const uint32_t t0 = shared::nowUs();
  while ((uint32_t)(shared::nowUs() - t0) < ms * 1000UL) {
    ImuSample s;
    if (!readSample(s)) continue;
    const float g[3] = {s.gyro.x, s.gyro.y, s.gyro.z};
    const float a[3] = {s.acc.x, s.acc.y, s.acc.z};
    for (int i = 0; i < 3; i++) {
      gs[i] += g[i];
      gq[i] += g[i] * g[i];
      as[i] += a[i];
    }
    n++;
  }
  if (n < 50) return false;                        // sensor not delivering

  for (int i = 0; i < 3; i++) {                    // moved during startup? keep the old state
    const float mean = gs[i] / n;
    const float var  = gq[i] / n - mean * mean;
    if (var > (STILL_DPS * DEG2RAD) * (STILL_DPS * DEG2RAD)) return false;
  }
  bias_ = {gs[0] / n, gs[1] / n, gs[2] / n};

  // roll / pitch from gravity, yaw = 0: this defines the world frame
  const float ax = as[0] / n, ay = as[1] / n, az = as[2] / n;
  const float roll  = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q_[0] = cr * cp;                                 // yaw = 0
  q_[1] = sr * cp;
  q_[2] = cr * sp;
  q_[3] = -sr * sp;

  vel_ = Vec3();
  accW_ = Vec3();
  haveLast_ = false;
  started_ = true;
  return true;
}

// ---------------------------------------------------------------- per sample

bool IMU::update() {
  ImuSample s;
  if (!readSample(s)) return false;
  step(s);
  return true;
}

void IMU::step(const ImuSample& s) {
  const float dt = haveLast_ ? (uint32_t)(s.tUs - last_.tUs) * 1e-6f : 0.0f;
  last_ = s;
  haveLast_ = true;
  rate_ = {s.gyro.x - bias_.x, s.gyro.y - bias_.y, s.gyro.z - bias_.z};

  if (dt > 0.0f && dt <= cfg_.maxDtS) {
    float wx = rate_.x, wy = rate_.y, wz = rate_.z;
    const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];

    // 1) roll / pitch correction from the accelerometer (complementary filter): turn the
    //    estimated "up" towards the measured one, only while the drone is not accelerating
    const float an = sqrtf(s.acc.x * s.acc.x + s.acc.y * s.acc.y + s.acc.z * s.acc.z);
    if (an > 1.0f - cfg_.accelGate && an < 1.0f + cfg_.accelGate) {
      const float mx = s.acc.x / an, my = s.acc.y / an, mz = s.acc.z / an;   // measured up
      const float vx = 2.0f * (x * z - w * y);                               // estimated up
      const float vy = 2.0f * (y * z + w * x);                               // (world z in body)
      const float vz = w * w - x * x - y * y + z * z;
      wx += cfg_.accelTrust * (my * vz - mz * vy);                           // measured x estimated
      wy += cfg_.accelTrust * (mz * vx - mx * vz);
      wz += cfg_.accelTrust * (mx * vy - my * vx);
    }

    // 2) integrate the attitude: q_dot = 0.5 * q * (0, w)
    const float h = 0.5f * dt;
    q_[0] = w + h * (-x * wx - y * wy - z * wz);
    q_[1] = x + h * (w * wx + y * wz - z * wy);
    q_[2] = y + h * (w * wy - x * wz + z * wx);
    q_[3] = z + h * (w * wz + x * wy - y * wx);
    const float norm = 1.0f / sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    for (int i = 0; i < 4; i++) q_[i] *= norm;
  }

  // 3) world acceleration: rotate into the world frame FIRST, then remove gravity (1 g up)
  const Vec3 aw = rotateQ(q_, s.acc);
  accW_ = {aw.x * GRAVITY, aw.y * GRAVITY, (aw.z - 1.0f) * GRAVITY};

  // 4) velocity, x / y only (z comes from the ToF)
  if (dt > 0.0f && dt <= cfg_.maxDtS) {
    vel_.x += accW_.x * dt;
    vel_.y += accW_.y * dt;
  }
}

// ---------------------------------------------------------------- results / helpers

Attitude IMU::attitude() const {
  const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
  float sp = 2.0f * (w * y - z * x);
  if (sp > 1.0f) sp = 1.0f;
  if (sp < -1.0f) sp = -1.0f;
  Attitude a;
  a.roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  a.pitch = asinf(sp);
  a.yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  return a;
}

void IMU::setVelocity(float vx, float vy) {
  vel_.x = vx;
  vel_.y = vy;
}

Vec3 IMU::rotate(const Attitude& a, const Vec3& v) {
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), world = R * body
  const float cr = cosf(a.roll), sr = sinf(a.roll);
  const float cp = cosf(a.pitch), sp = sinf(a.pitch);
  const float cy = cosf(a.yaw), sy = sinf(a.yaw);
  Vec3 o;
  o.x = cy * cp * v.x + (cy * sp * sr - sy * cr) * v.y + (cy * sp * cr + sy * sr) * v.z;
  o.y = sy * cp * v.x + (sy * sp * sr + cy * cr) * v.y + (sy * sp * cr - cy * sr) * v.z;
  o.z = -sp * v.x + cp * sr * v.y + cp * cr * v.z;
  return o;
}

Vec3 IMU::rotateQ(const float q[4], const Vec3& v) {
  // v' = q v q*, as t = 2 (q_xyz x v), v' = v + w t + q_xyz x t
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float tx = 2.0f * (y * v.z - z * v.y);
  const float ty = 2.0f * (z * v.x - x * v.z);
  const float tz = 2.0f * (x * v.y - y * v.x);
  Vec3 o;
  o.x = v.x + w * tx + (y * tz - z * ty);
  o.y = v.y + w * ty + (z * tx - x * tz);
  o.z = v.z + w * tz + (x * ty - y * tx);
  return o;
}

bool IMU::readSample(ImuSample& s) {
  uint8_t st = 0;
  readRegs(REG_STATUS, &st, 1);
  if (!(st & 0x02)) return false;                  // no new gyro sample (accel has the same rate)

  int16_t d[6];                                    // gyro x y z, accel x y z, one burst
  readRegs(REG_OUTX_L_G, (uint8_t*)d, 12);
  s.tUs = shared::nowUs();

  // gyro: scale only, the bias is removed in step() (startup() measures it)
  const float gs = GYRO_LSB * DEG2RAD;
  s.gyro = {d[0] * gs * calib::GYRO_SCALE[0], d[1] * gs * calib::GYRO_SCALE[1],
            d[2] * gs * calib::GYRO_SCALE[2]};

  // accelerometer: the full calibration fit if there is one, else bias + scale
  const float ar[3] = {d[3] * ACC_LSB, d[4] * ACC_LSB, d[5] * ACC_LSB};
  float a[3];
  if (calib::ACC_HAS_FIT) {
    const float b[3] = {ar[0] - calib::ACC_BIAS_FIT_G[0], ar[1] - calib::ACC_BIAS_FIT_G[1],
                        ar[2] - calib::ACC_BIAS_FIT_G[2]};
    for (int i = 0; i < 3; i++)
      a[i] = calib::ACC_M[i][0] * b[0] + calib::ACC_M[i][1] * b[1] + calib::ACC_M[i][2] * b[2];
  } else {
    for (int i = 0; i < 3; i++) a[i] = (ar[i] - calib::ACC_BIAS_G[i]) / calib::ACC_SCALE[i];
  }
  s.acc = {a[0], a[1], a[2]};
  // body axes = IMU axes for now; if the board sits rotated on the drone, map the axes here
  return true;
}

void IMU::writeReg(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(imuSpi);
  digitalWrite(CS, LOW);
  SPI1.transfer(reg);
  SPI1.transfer(val);
  digitalWrite(CS, HIGH);
  SPI1.endTransaction();
}

void IMU::readRegs(uint8_t reg, uint8_t* buf, int n) {
  SPI1.beginTransaction(imuSpi);
  digitalWrite(CS, LOW);
  SPI1.transfer(0x80 | reg);                       // bit 7 = read
  for (int i = 0; i < n; i++) buf[i] = SPI1.transfer(0);
  digitalWrite(CS, HIGH);
  SPI1.endTransaction();
}

}  // namespace sensors
