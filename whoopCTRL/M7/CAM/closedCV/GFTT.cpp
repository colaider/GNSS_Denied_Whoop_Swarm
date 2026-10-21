#include "headers/GFTT.h"

#include <math.h>

#include <algorithm>

namespace ccv {

GFTT::GFTT(int maxFeatures, float quality, float minDistance, float minScore, int border)
    : maxFeatures_(maxFeatures),
      quality_(quality),
      minDist2_(minDistance * minDistance),
      minScore_(minScore),
      border_(border < 4 ? 4 : border) {}   // the 5x5 window + gradients need 3 px, +1 neighbour

int GFTT::detect(const Image& img, Point2f* out, const Rect* roi,
                 const Point2f* existing, int nExisting, int maxOut) {
  const int limit = (maxOut > 0 && maxOut < maxFeatures_) ? maxOut : maxFeatures_;
  nCand_ = 0;

  // search area [x0, x1) x [y0, y1): the roi, kept away from the border
  int x0 = border_, y0 = border_;
  int x1 = img.width - border_, y1 = img.height - border_;
  if (roi) {
    if (roi->x > x0) x0 = roi->x;
    if (roi->y > y0) y0 = roi->y;
    if (roi->x + roi->w < x1) x1 = roi->x + roi->w;
    if (roi->y + roi->h < y1) y1 = roi->y + roi->h;
  }
  const int rw = x1 - x0 + 2;                      // a score row, plus one neighbour each side
  if (x1 <= x0 || y1 <= y0 || rw > MAX_ROW) return 0;

  // 1) score rows one by one; once 3 rows exist, keep the local maxima of the middle one
  for (int y = y0 - 1; y <= y1; y++) {
    float* row = rows_[y % 3];
    for (int i = 0; i < rw; i++) row[i] = score(img, x0 - 1 + i, y);

    if (y < y0 + 1) continue;
    const int yc = y - 1;                          // middle row, its neighbours are complete
    const float* up  = rows_[(yc - 1) % 3];
    const float* mid = rows_[yc % 3];
    const float* dn  = row;
    for (int i = 1; i < rw - 1; i++) {
      const float s = mid[i];
      if (s < minScore_) continue;
      if (s < mid[i - 1] || s < mid[i + 1] || s < up[i - 1] || s < up[i] || s < up[i + 1] ||
          s < dn[i - 1] || s < dn[i] || s < dn[i + 1])
        continue;
      addCandidate(x0 - 1 + i, yc, s);
    }
  }
  if (nCand_ == 0) return 0;

  // 2) strongest first, relative quality threshold
  std::sort(cand_, cand_ + nCand_,
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
  const float thr = quality_ * cand_[0].score > minScore_ ? quality_ * cand_[0].score : minScore_;

  // 3) greedy pick with minimum spacing to existing and already chosen points
  int n = 0;
  for (int k = 0; k < nCand_ && n < limit; k++) {
    if (cand_[k].score < thr) break;
    const float px = cand_[k].x, py = cand_[k].y;
    bool tooClose = false;
    for (int j = 0; j < nExisting && !tooClose; j++) {
      const float dx = px - existing[j].x, dy = py - existing[j].y;
      tooClose = dx * dx + dy * dy < minDist2_;
    }
    for (int j = 0; j < n && !tooClose; j++) {
      const float dx = px - out[j].x, dy = py - out[j].y;
      tooClose = dx * dx + dy * dy < minDist2_;
    }
    if (tooClose) continue;
    out[n].x = px;
    out[n].y = py;
    n++;
  }
  return n;
}

float GFTT::score(const Image& img, int x, int y) {
  // G summed over the 5x5 window around (x, y). Gradients without the /2 of the central
  // difference (integers); the /4 is put back on the squares below.
  const int w = img.width;
  int32_t a = 0, b = 0, c = 0;
  for (int dy = -2; dy <= 2; dy++) {
    const uint8_t* r = img.data + (y + dy) * w + x;
    for (int dx = -2; dx <= 2; dx++) {
      const int gx = r[dx + 1] - r[dx - 1];
      const int gy = r[dx + w] - r[dx - w];
      a += gx * gx;
      b += gy * gy;
      c += gx * gy;
    }
  }
  const float fa = a * 0.25f, fb = b * 0.25f, fc = c * 0.25f;
  const float half = 0.5f * (fa - fb);
  return 0.5f * (fa + fb) - sqrtf(half * half + fc * fc);   // smallest eigenvalue
}

void GFTT::addCandidate(int x, int y, float s) {
  if (nCand_ < MAX_CANDIDATES) {
    cand_[nCand_++] = {(int16_t)x, (int16_t)y, s};
    return;
  }
  int weakest = 0;                                 // full: replace the weakest if this is stronger
  for (int k = 1; k < MAX_CANDIDATES; k++)
    if (cand_[k].score < cand_[weakest].score) weakest = k;
  if (s > cand_[weakest].score) cand_[weakest] = {(int16_t)x, (int16_t)y, s};
}

}  // namespace ccv
