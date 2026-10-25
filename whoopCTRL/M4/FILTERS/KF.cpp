#include "headers/KF.h"

#include <math.h>

namespace filters {

// ---------------- horizontal: [px, py, vx, vy, bx, by] ----------------

void HorizontalKF::reset(uint32_t tUs) {
  Gaussian<6> g;                                   // x = 0: at rest at the origin
  g.P[PX][PX] = g.P[PY][PY] = 1e-6f;               // the origin is defined here: (1 mm)^2
  g.P[VX][VX] = g.P[VY][VY] = 0.01f * 0.01f;       // still at startup
  g.P[BX][BX] = g.P[BY][BY] = cfg_.biasInit * cfg_.biasInit;
  DelayedKF::reset(tUs, g);
  innov_[0] = innov_[1] = innovVar_[0] = innovVar_[1] = 0.0f;
  forced_ = 0;
}

void HorizontalKF::predict(uint32_t tUs, float ax, float ay, float yaw, float k) {
  const float u[4] = {ax, ay, yaw, k};
  DelayedKF::predict(tUs, u);
}

// The measured acceleration minus the bias (turned into the world with the yaw), integrated:
//   a  = acc - R(yaw) b
//   p += v dt + a dt^2 / 2,   v += a dt,   b stays (random walk)
void HorizontalKF::model(float dt, const float u[4], float F[6][6], float b[6], float Q[6][6]) const {
  const float c = cosf(u[2]), s = sinf(u[2]), h = 0.5f * dt * dt;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) { F[i][j] = (i == j) ? 1.0f : 0.0f; Q[i][j] = 0.0f; }
  F[PX][VX] = dt;      F[PY][VY] = dt;
  F[PX][BX] = -c * h;  F[PX][BY] =  s * h;         // the bias' world x part: c bx - s by
  F[PY][BX] = -s * h;  F[PY][BY] = -c * h;         // world y part:            s bx + c by
  F[VX][BX] = -c * dt; F[VX][BY] =  s * dt;
  F[VY][BX] = -s * dt; F[VY][BY] = -c * dt;
  b[PX] = u[0] * h;  b[PY] = u[1] * h;
  b[VX] = u[0] * dt; b[VY] = u[1] * dt;
  b[BX] = b[BY] = 0.0f;

  // white acceleration noise q integrated over dt (per axis: position, velocity and their link)
  const float q = cfg_.accNoise * cfg_.accNoise * u[3] * u[3];
  for (int a = 0; a < 2; a++) {
    const int p = PX + a, v = VX + a;
    Q[p][p] = q * dt * dt * dt / 3.0f;
    Q[p][v] = Q[v][p] = q * dt * dt / 2.0f;
    Q[v][v] = q * dt;
  }
  Q[BX][BX] = Q[BY][BY] = cfg_.biasWalk * cfg_.biasWalk * dt;
}

int HorizontalKF::updateVelocity(uint32_t tUs, float vx, float vy, float std) {
  const float H[2][6] = {{0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}};
  const float z[2] = {vx, vy}, R[2] = {std * std, std * std};
  const int n = update(tUs, 2, H, z, R, cfg_.gate, innov_, innovVar_);
  if (n == 0 && rejects() >= cfg_.forceAfter) {    // the camera keeps disagreeing: it is right
    setVelocity(tUs, vx, vy, std);
    forced_++;
  }
  return n;
}

bool HorizontalKF::setVelocity(uint32_t tUs, float vx, float vy, float std) {
  const int idx[2] = {VX, VY};
  const float z[2] = {vx, vy}, R[2] = {std * std, std * std};
  return force(tUs, 2, idx, z, R);
}

void HorizontalKF::zeroVelocity(uint32_t tUs, float std) {
  const float H[2][6] = {{0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}};
  const float z[2] = {0.0f, 0.0f}, R[2] = {std * std, std * std};
  update(tUs, 2, H, z, R, 0.0f, nullptr, nullptr, false);   // never refused, outside the time order;
                                                            // the camera's innovations stay as they were
}

float HorizontalKF::innovationStd(int axis) const { return sqrtf(innovVar_[axis]); }

// ---------------- vertical: [z, vz, bz] ----------------

void VerticalKF::reset(uint32_t tUs, float z) {
  Gaussian<3> g;
  g.x[Z] = z;
  g.P[Z][Z]   = 0.01f * 0.01f;
  g.P[VZ][VZ] = 0.01f * 0.01f;
  g.P[BZ][BZ] = cfg_.biasInit * cfg_.biasInit;
  DelayedKF::reset(tUs, g);
  innov_ = innovVar_ = 0.0f;
  steps_ = 0;
}

void VerticalKF::predict(uint32_t tUs, float az, float k) {
  const float u[2] = {az, k};
  DelayedKF::predict(tUs, u);
}

//   a = acc - bz;   z += vz dt + a dt^2 / 2,   vz += a dt
void VerticalKF::model(float dt, const float u[2], float F[3][3], float b[3], float Q[3][3]) const {
  const float h = 0.5f * dt * dt, q = cfg_.accNoise * cfg_.accNoise * u[1] * u[1];
  const float f[3][3] = {{1, dt, -h}, {0, 1, -dt}, {0, 0, 1}};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) { F[i][j] = f[i][j]; Q[i][j] = 0.0f; }
  b[Z] = u[0] * h;  b[VZ] = u[0] * dt;  b[BZ] = 0.0f;
  Q[Z][Z]   = q * dt * dt * dt / 3.0f;
  Q[Z][VZ]  = Q[VZ][Z] = q * dt * dt / 2.0f;
  Q[VZ][VZ] = q * dt;
  Q[BZ][BZ] = cfg_.biasWalk * cfg_.biasWalk * dt;
}

int VerticalKF::updateHeight(uint32_t tUs, float z, float std) {
  const float H[1][3] = {{1, 0, 0}};
  const float R[1] = {std * std};
  const int n = update(tUs, 1, H, &z, R, cfg_.gate, &innov_, &innovVar_);
  if (n == 0 && rejects() >= cfg_.forceAfter) {    // the floor moved (a step, a table edge)
    const int idx[1] = {Z};
    force(tUs, 1, idx, &z, R);
    steps_++;
    return 2;
  }
  return n;
}

bool VerticalKF::setHeight(uint32_t tUs, float z, float std) {
  const int idx[1] = {Z};
  const float R[1] = {std * std};
  return force(tUs, 1, idx, &z, R);
}

void VerticalKF::zeroVelocity(uint32_t tUs, float std) {
  const float H[1][3] = {{0, 1, 0}};
  const float z[1] = {0.0f}, R[1] = {std * std};
  update(tUs, 1, H, z, R, 0.0f, nullptr, nullptr, false);
}

float VerticalKF::innovationStd() const { return sqrtf(innovVar_); }

}  // namespace filters
