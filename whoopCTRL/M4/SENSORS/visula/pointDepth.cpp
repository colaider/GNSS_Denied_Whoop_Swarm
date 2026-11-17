#include "headers/pointDepth.h"

#include <math.h>

namespace sensors {

void PointDepth::update(const Sensors& s, const msg::FlowHeader& f, const msg::PointMsg* pts, int n) {
  const float range = s.tof().lastValid().rangeM;
  if (range <= 0.0f || f.fx <= 0.0f || f.fy <= 0.0f) return;
  const Sensors::CameraDebug& c = s.cameraDebug();
  const float dt = (f.t1Us - f.t0Us) * 1e-6f;
  const bool  moved = s.cameraValid() && dt > 0.0f && dt < 0.2f;   // the camera's move over this pair is known
  const float tx = c.vx * dt, ty = c.vy * dt, sigma = cfg_.pxNoise / f.fx;
  const Pose  pose = s.pose();
  const Attitude att = s.attitude();

  Track next[MAXP];
  int m = 0;
  for (int i = 0; i < n && m < MAXP; i++) {
    const msg::PointMsg& q = pts[i];
    Track t{q.id, 1.0f / range, 0.0f};
    t.P = cfg_.initRel * t.rho * cfg_.initRel * t.rho;
    for (int k = 0; k < n_; k++) {
      if (tr_[k].id != q.id) continue;
      t = tr_[k];
      const float z = 1.0f / t.rho + c.dZ;      // the camera came closer by -dZ: so did the point
      if (z > cfg_.minZ) t.rho = 1.0f / z;
      break;
    }

    if (moved) {                                // this pair's parallax: m = g / Z
      const float xn = (q.x - 0.5f * q.dx - f.cx) / f.fx, yn = (q.y - 0.5f * q.dy - f.cy) / f.fy;
      const Vec3& r = c.rot;
      const float mx = q.dx / f.fx - (xn * yn * r.x - (1.0f + xn * xn) * r.y + yn * r.z);
      const float my = q.dy / f.fy - ((1.0f + yn * yn) * r.x - xn * yn * r.y - xn * r.z);
      const float gx = -(tx + xn * c.dZ), gy = -(ty + yn * c.dZ);
      const float gg = gx * gx + gy * gy;
      if (gg > 0.0f) {
        const float meas = (gx * mx + gy * my) / gg, var = sigma * sigma / gg, d = meas - t.rho;
        if (d * d <= cfg_.gate * cfg_.gate * (t.P + var)) {
          const float K = t.P / (t.P + var);
          t.rho += K * d;
          t.P   *= 1.0f - K;
        }
      }
    }
    if (t.rho > 1.0f / cfg_.minZ) t.rho = 1.0f / cfg_.minZ;
    if (t.rho < 1.0f / cfg_.maxZ) t.rho = 1.0f / cfg_.maxZ;
    next[m] = t;

    Point& o = out_[m];                         // into the start frame: camera ray x depth, turned like the drone
    const float z = 1.0f / t.rho;
    const Vec3 cam{(q.x - f.cx) / f.fx * z, (q.y - f.cy) / f.fy * z, z};
    const Vec3 w = IMU::rotate(att, s.toBody(cam));
    o.id = q.id;  o.x = q.x;  o.y = q.y;
    o.depth = z;
    o.std = sqrtf(t.P) * z * z;
    o.world = {pose.x + w.x, pose.y + w.y, pose.z + w.z};
    m++;
  }
  for (int k = 0; k < m; k++) tr_[k] = next[k];
  n_ = m;
}

}  // namespace sensors
