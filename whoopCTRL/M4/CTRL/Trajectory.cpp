#include "headers/Trajectory.h"

namespace ctrl {

bool Trajectory::start(uint16_t id, uint32_t startUs) {
  if (id == id_) return false;                                // the PC sends it a few times
  id_ = id;  startUs_ = startUs;  n_ = 0;
  return true;
}

void Trajectory::add(uint16_t id, const msg::TrajPoint* pts, int n) {
  if (id_ == 0 || id != id_) return;
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

bool Trajectory::sample(uint32_t nowUs, sensors::Pose& p, sensors::Pose& v, sensors::Pose& a) {
  const float t = (int32_t)(nowUs - startUs_) * 1e-6f;       // trajectory time, s
  while (n_ >= 2 && pts_[1].tMs * 1e-3f <= t) drop();         // pts_[0]: the last point at or before t
  if (!eval(t, p, v, a)) return false;

  sensors::Pose pl, vl, al;                                   // the acceleration ahead: the tilt / thrust arrives
  if (leadS > 0.0f && eval(t + leadS, pl, vl, al)) { a.x = al.x;  a.y = al.y; }   // that much later (beyond the
  if (leadZS > 0.0f && eval(t + leadZS, pl, vl, al)) a.z = al.z;                  // points: as at t)
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
