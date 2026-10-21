#include "headers/RANSAC.h"

#include <math.h>

namespace ccv {

RANSAC::RANSAC(int iterations, float threshold, int minInliers, float minSampleDist)
    : iterations_(iterations < 1 ? 1 : iterations),
      thr2_(threshold * threshold),
      minInliers_(minInliers < 2 ? 2 : minInliers),
      minDist2_(minSampleDist * minSampleDist) {}

bool RANSAC::estimate(const Point2f* p, const Point2f* q, int n, Similarity& out,
                      uint8_t* inlier) {
  out = Similarity();
  if (inlier)
    for (int k = 0; k < n; k++) inlier[k] = 0;
  if (n < minInliers_) return false;

  // 1) random 2-point models, keep the one with the most inliers (ties: smaller error)
  Similarity best;
  int   bestCount = 0;
  float bestErr   = 0;
  for (int it = 0; it < iterations_; it++) {
    const int i = (int)(random() % (uint32_t)n);
    const int j = (int)(random() % (uint32_t)n);
    const float dpx = p[j].x - p[i].x, dpy = p[j].y - p[i].y;
    const float dqx = q[j].x - q[i].x, dqy = q[j].y - q[i].y;
    const float d2 = dpx * dpx + dpy * dpy;
    if (i == j || d2 < minDist2_) continue;       // too close: a, b would be unreliable

    Similarity m;                                  // dq = [a -b; b a] dp, translation cancels
    m.a  = (dpx * dqx + dpy * dqy) / d2;
    m.b  = (dpx * dqy - dpy * dqx) / d2;
    m.tx = q[i].x - (m.a * p[i].x - m.b * p[i].y);
    m.ty = q[i].y - (m.b * p[i].x + m.a * p[i].y);

    float err;
    const int c = countInliers(p, q, n, m, &err);
    if (c > bestCount || (c == bestCount && c > 0 && err < bestErr)) {
      best = m;
      bestCount = c;
      bestErr = err;
    }
  }
  if (bestCount < 2) return false;

  // 2) least squares on the inliers, twice (the refit can change who is an inlier)
  refit(p, q, n, best);
  refit(p, q, n, best);

  // 3) final inliers, error and the readable values
  float sumErr2;
  best.nInliers = countInliers(p, q, n, best, &sumErr2);
  best.rms   = best.nInliers ? sqrtf(sumErr2 / best.nInliers) : 0.0f;
  best.scale = sqrtf(best.a * best.a + best.b * best.b);
  best.angle = atan2f(best.b, best.a);
  if (inlier) {
    for (int k = 0; k < n; k++) {
      const float ex = q[k].x - (best.a * p[k].x - best.b * p[k].y + best.tx);
      const float ey = q[k].y - (best.b * p[k].x + best.a * p[k].y + best.ty);
      inlier[k] = (ex * ex + ey * ey < thr2_) ? 1 : 0;
    }
  }
  out = best;
  return best.nInliers >= minInliers_;
}

int RANSAC::countInliers(const Point2f* p, const Point2f* q, int n, const Similarity& m,
                         float* sumErr2) const {
  int count = 0;
  float sum = 0;
  for (int k = 0; k < n; k++) {
    const float ex = q[k].x - (m.a * p[k].x - m.b * p[k].y + m.tx);
    const float ey = q[k].y - (m.b * p[k].x + m.a * p[k].y + m.ty);
    const float r2 = ex * ex + ey * ey;
    if (r2 < thr2_) {
      count++;
      sum += r2;
    }
  }
  if (sumErr2) *sumErr2 = sum;
  return count;
}

bool RANSAC::refit(const Point2f* p, const Point2f* q, int n, Similarity& m) const {
  // centroids of the current inliers
  float spx = 0, spy = 0, sqx = 0, sqy = 0;
  int c = 0;
  for (int k = 0; k < n; k++) {
    const float ex = q[k].x - (m.a * p[k].x - m.b * p[k].y + m.tx);
    const float ey = q[k].y - (m.b * p[k].x + m.a * p[k].y + m.ty);
    if (ex * ex + ey * ey >= thr2_) continue;
    spx += p[k].x; spy += p[k].y; sqx += q[k].x; sqy += q[k].y;
    c++;
  }
  if (c < 2) return false;
  const float mpx = spx / c, mpy = spy / c, mqx = sqx / c, mqy = sqy / c;

  // a, b from the centred points; t from the centroids
  float num_a = 0, num_b = 0, den = 0;
  for (int k = 0; k < n; k++) {
    const float ex = q[k].x - (m.a * p[k].x - m.b * p[k].y + m.tx);
    const float ey = q[k].y - (m.b * p[k].x + m.a * p[k].y + m.ty);
    if (ex * ex + ey * ey >= thr2_) continue;
    const float px = p[k].x - mpx, py = p[k].y - mpy;
    const float qx = q[k].x - mqx, qy = q[k].y - mqy;
    num_a += px * qx + py * qy;
    num_b += px * qy - py * qx;
    den   += px * px + py * py;
  }
  if (den < 1e-6f) return false;

  m.a  = num_a / den;
  m.b  = num_b / den;
  m.tx = mqx - (m.a * mpx - m.b * mpy);
  m.ty = mqy - (m.b * mpx + m.a * mpy);
  return true;
}

uint32_t RANSAC::random() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return rng_;
}

}  // namespace ccv
