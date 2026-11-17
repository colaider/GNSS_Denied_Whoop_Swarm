#include "headers/Trajectory.h"

namespace ctrl {

const char* Trajectory::stateName() const {
  static const char* const names[] = {"IDLE", "WAITING", "PLAYING", "EMPTY", "HOLD"};
  return names[state_];
}

void Trajectory::start(uint16_t id, uint32_t startUs) {
  if (state_ != IDLE && id == id_) return;                    // the PC sends it a few times
  clear();
  id_ = id;  startUs_ = startUs;  state_ = WAITING;
}

void Trajectory::add(uint16_t id, const msg::TrajPoint* pts, int n) {
  if (state_ == IDLE || id != id_) return;
  for (int i = 0; i < n; i++) {
    const msg::TrajPoint& q = pts[i];
    int k = n_;
    while (k > 0 && pts_[k - 1].tMs >= q.tMs) k--;            // k: the first point not before q
    if (k < n_ && pts_[k].tMs == q.tMs) { pts_[k] = q; continue; }   // same time: the newer plan
    if (n_ == MAX_POINTS) { if (k == 0) continue; drop(); k--; }
    for (int j = n_; j > k; j--) pts_[j] = pts_[j - 1];
    pts_[k] = q;
    n_++;
  }
}

void Trajectory::drop() {
  for (int j = 1; j < n_; j++) pts_[j - 1] = pts_[j];
  n_--;
}

bool Trajectory::update(uint32_t nowUs) {
  if (state_ == IDLE) return false;
  v_ = a_ = sensors::Pose();
  if (state_ == HOLD) return true;

  const float t = (int32_t)(nowUs - startUs_) * 1e-6f;       // trajectory time, s
  while (n_ >= 2 && pts_[1].tMs * 1e-3f <= t) drop();         // pts_[0]: the last point at or before t
  if (n_ == 0 || t < pts_[0].tMs * 1e-3f) { state_ = n_ || t < 0.0f ? WAITING : EMPTY; return true; }   // keep p_
  const msg::TrajPoint& a = pts_[0];
  if (n_ == 1) {                                              // no point after it yet: wait there
    p_ = {a.x, a.y, a.z, a.yaw};
    state_ = EMPTY;
    return true;
  }

  eval(t, p_, v_, a_);
  sensors::Pose pl, vl, al;                                   // the acceleration leadS ahead: the tilt arrives
  if (leadS > 0.0f && eval(t + leadS, pl, vl, al)) a_ = al;   // that much later (beyond the points: as at t)
  state_ = PLAYING;
  return true;
}

bool Trajectory::eval(float t, sensors::Pose& p, sensors::Pose& v, sensors::Pose& acc) const {
  int i = 0;
  while (i + 1 < n_ && pts_[i + 1].tMs * 1e-3f <= t) i++;
  if (i + 1 >= n_ || t < pts_[i].tMs * 1e-3f) return false;

  // cubic Hermite: through both points with their velocities
  const msg::TrajPoint& a = pts_[i];
  const msg::TrajPoint& b = pts_[i + 1];
  const float h = (b.tMs - a.tMs) * 1e-3f, s = (t - a.tMs * 1e-3f) / h, s2 = s * s, s3 = s2 * s;
  const sensors::Pose p0{a.x, a.y, a.z, 0.0f}, p1{b.x, b.y, b.z, 0.0f};
  const sensors::Pose m0 = sensors::Pose{a.vx, a.vy, a.vz, 0.0f} * h, m1 = sensors::Pose{b.vx, b.vy, b.vz, 0.0f} * h;
  p   = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (3 * s2 - 2 * s3) * p1 + (s3 - s2) * m1;
  v   = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (6 * s - 6 * s2) * p1 + (3 * s2 - 2 * s) * m1) / h;
  acc = ((12 * s - 6) * p0 + (6 * s - 4) * m0 + (6 - 12 * s) * p1 + (6 * s - 2) * m1) / (h * h);

  const float dYaw = sensors::wrapPi(b.yaw - a.yaw);          // yaw: straight from one point to the next
  p.yaw = sensors::wrapPi(a.yaw + s * dYaw);
  v.yaw = dYaw / h;
  acc.yaw = 0.0f;
  return true;
}

}
