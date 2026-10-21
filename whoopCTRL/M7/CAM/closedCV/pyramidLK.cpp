#include "headers/pyramidLK.h"

#include <math.h>
#include <string.h>

namespace ccv {

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

PyramidLK::PyramidLK(int levels, int winSize, int maxIter, float epsilon, float minEigen,
                     float fbThreshold)
    : levels_(levels < 1 ? 1 : (levels > MAX_LEVELS ? MAX_LEVELS : levels)),
      maxIter_(maxIter < 1 ? 1 : maxIter),
      eps2_(epsilon * epsilon),
      minEigen_(minEigen),
      fb2_(fbThreshold * fbThreshold) {
  int w = winSize | 1;                             // odd
  if (w < 3) w = 3;
  if (w > MAX_WIN) w = MAX_WIN;
  win_  = w;
  half_ = w / 2;
}

bool PyramidLK::newFrame(const Image& img) {
  if (!img.data || img.width > MAX_W || img.height > MAX_H) return false;
  cur_ ^= 1;                                       // the old "newest" becomes "previous"
  buildPyramid(pyr_[cur_], store_[cur_], img);
  if (frames_ < 2) frames_++;
  return true;
}

int PyramidLK::track(const Point2f* prevPts, Point2f* curPts, uint8_t* status, int n,
                     const Point2f* guess) {
  int ok = 0;
  const Pyramid& prev = pyr_[cur_ ^ 1];
  const Pyramid& cur  = pyr_[cur_];
  for (int i = 0; i < n; i++) {
    status[i] = 0;
    curPts[i] = prevPts[i];
    if (!ready()) continue;

    const Point2f g = guess ? guess[i] : Point2f();
    Point2f q;
    if (!trackPoint(prev, cur, prevPts[i], g, q)) continue;

    // forward-backward: track back and require landing near the start point
    Point2f back;
    const Point2f gb = {prevPts[i].x - q.x, prevPts[i].y - q.y};
    if (!trackPoint(cur, prev, q, gb, back)) continue;
    const float ex = back.x - prevPts[i].x, ey = back.y - prevPts[i].y;
    if (ex * ex + ey * ey > fb2_) continue;

    curPts[i] = q;
    status[i] = 1;
    ok++;
  }
  return ok;
}

void PyramidLK::buildPyramid(Pyramid& py, uint8_t* mem, const Image& img) {
  py.lv[0].data = mem;
  py.lv[0].w    = img.width;
  py.lv[0].h    = img.height;
  memcpy(mem, img.data, (size_t)img.width * img.height);
  mem += img.width * img.height;

  // each level: 5-tap Gaussian [1 4 6 4 1]/16 in x and y, then keep every second pixel.
  // The blur widens the motion range of the coarse levels.
  static const int K[5] = {1, 4, 6, 4, 1};
  for (int L = 1; L < levels_; L++) {
    const Level& s = py.lv[L - 1];
    Level& d = py.lv[L];
    d.data = mem;
    d.w = s.w / 2;
    d.h = s.h / 2;
    for (int y = 0; y < d.h; y++) {
      for (int x = 0; x < d.w; x++) {
        int sum = 0;
        for (int j = -2; j <= 2; j++) {
          const uint8_t* r = s.data + clampi(2 * y + j, 0, s.h - 1) * s.w;
          int rs = 0;
          for (int i = -2; i <= 2; i++) rs += K[i + 2] * r[clampi(2 * x + i, 0, s.w - 1)];
          sum += K[j + 2] * rs;
        }
        d.data[y * d.w + x] = (uint8_t)((sum + 128) >> 8);   // /256
      }
    }
    mem += d.w * d.h;
  }
}

void PyramidLK::sample(const Level& img, float x0, float y0, int size, float* out) {
  // size x size bilinear samples starting at (x0, y0). The fractional part is the same for
  // every sample, so the 4 weights are computed once. Outside the image the edge pixels are
  // repeated.
  const int xi = (int)floorf(x0), yi = (int)floorf(y0);
  const float ax = x0 - xi, ay = y0 - yi;
  const float w00 = (1 - ax) * (1 - ay), w10 = ax * (1 - ay);
  const float w01 = (1 - ax) * ay, w11 = ax * ay;

  if (xi >= 0 && yi >= 0 && xi + size < img.w && yi + size < img.h) {   // fully inside: fast
    for (int j = 0; j < size; j++) {
      const uint8_t* r0 = img.data + (yi + j) * img.w + xi;
      const uint8_t* r1 = r0 + img.w;
      for (int i = 0; i < size; i++)
        out[j * size + i] = w00 * r0[i] + w10 * r0[i + 1] + w01 * r1[i] + w11 * r1[i + 1];
    }
    return;
  }
  for (int j = 0; j < size; j++) {                 // touches the border: clamp coordinates
    const uint8_t* r0 = img.data + clampi(yi + j, 0, img.h - 1) * img.w;
    const uint8_t* r1 = img.data + clampi(yi + j + 1, 0, img.h - 1) * img.w;
    for (int i = 0; i < size; i++) {
      const int xa = clampi(xi + i, 0, img.w - 1), xb = clampi(xi + i + 1, 0, img.w - 1);
      out[j * size + i] = w00 * r0[xa] + w10 * r0[xb] + w01 * r1[xa] + w11 * r1[xb];
    }
  }
}

bool PyramidLK::trackPoint(const Pyramid& I, const Pyramid& J, Point2f p, Point2f guess,
                           Point2f& q) {
  const int P = win_ + 2;                          // template incl. a 1 px border for gradients

  // top level: skip levels smaller than the window
  int top = levels_ - 1;
  while (top > 0 && (I.lv[top].w < P || I.lv[top].h < P)) top--;

  // motion guess at the top level (motions scale by 1/2 per level)
  float gx = guess.x / (float)(1 << top);
  float gy = guess.y / (float)(1 << top);

  for (int L = top; L >= 0; L--) {
    const Level& A = I.lv[L];
    const Level& B = J.lv[L];
    const float inv = 1.0f / (float)(1 << L);
    const float px = p.x * inv, py = p.y * inv;    // point at this level

    // template window + gradients, and G (once per level)
    sample(A, px - half_ - 1, py - half_ - 1, P, patch_);
    float Gxx = 0, Gxy = 0, Gyy = 0;
    int k = 0;
    for (int j = 1; j <= win_; j++) {
      for (int i = 1; i <= win_; i++, k++) {
        const float ix = 0.5f * (patch_[j * P + i + 1] - patch_[j * P + i - 1]);
        const float iy = 0.5f * (patch_[(j + 1) * P + i] - patch_[(j - 1) * P + i]);
        gx_[k] = ix;
        gy_[k] = iy;
        Gxx += ix * ix;
        Gxy += ix * iy;
        Gyy += iy * iy;
      }
    }
    const float half = 0.5f * (Gxx - Gyy);
    const float lmin = 0.5f * (Gxx + Gyy) - sqrtf(half * half + Gxy * Gxy);
    const float det  = Gxx * Gyy - Gxy * Gxy;
    if (lmin < minEigen_ * win_ * win_ || det <= 0.0f) return false;   // flat or edge
    const float iXX = Gyy / det, iXY = -Gxy / det, iYY = Gxx / det;    // G^-1

    // iterate: G * eta = b,  b = sum (I - J(shifted)) * grad I
    float vx = 0, vy = 0;
    for (int it = 0; it < maxIter_; it++) {
      sample(B, px + gx + vx - half_, py + gy + vy - half_, win_, jwin_);
      float bx = 0, by = 0;
      k = 0;
      for (int j = 0; j < win_; j++) {
        const float* t = patch_ + (j + 1) * P + 1;
        for (int i = 0; i < win_; i++, k++) {
          const float d = t[i] - jwin_[k];
          bx += d * gx_[k];
          by += d * gy_[k];
        }
      }
      const float ex = iXX * bx + iXY * by;
      const float ey = iXY * bx + iYY * by;
      vx += ex;
      vy += ey;
      if (fabsf(vx) > 1000.0f || fabsf(vy) > 1000.0f) return false;   // diverged
      if (ex * ex + ey * ey < eps2_) break;
    }

    if (L > 0) {                                   // pass the estimate down, doubled
      gx = 2.0f * (gx + vx);
      gy = 2.0f * (gy + vy);
    } else {
      q.x = p.x + gx + vx;
      q.y = p.y + gy + vy;
    }
  }
  // lost if it ended up outside the image
  return q.x >= 0 && q.y >= 0 && q.x <= I.lv[0].w - 1 && q.y <= I.lv[0].h - 1;
}

}  // namespace ccv
