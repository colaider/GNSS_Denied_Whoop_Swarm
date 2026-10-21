#include "headers/replenish.h"

namespace ccv {

Replenish::Replenish(GFTT& gftt, int maxPoints, int sectorsX, int sectorsY, int maxStrikes,
                     float mergeDistance)
    : gftt_(gftt),
      maxPoints_(maxPoints < 1 ? 1 : (maxPoints > MAX_POINTS ? MAX_POINTS : maxPoints)),
      sx_(sectorsX < 1 ? 1 : (sectorsX > 4 ? 4 : sectorsX)),
      sy_(sectorsY < 1 ? 1 : (sectorsY > 4 ? 4 : sectorsY)),
      maxStrikes_(maxStrikes < 1 ? 1 : maxStrikes),
      merge2_(mergeDistance * mergeDistance) {}

void Replenish::update(const Point2f* newPts, const uint8_t* status, const uint8_t* inlier) {
  // 1) move survivors, drop lost points and repeated outliers. Walk backwards so removing
  //    point i does not shift the entries still to be read (they share the old order).
  for (int i = n_ - 1; i >= 0; i--) {
    if (!status[i]) {
      remove(i);
      continue;
    }
    pts_[i] = newPts[i];
    if (inlier) {
      strikes_[i] = inlier[i] ? 0 : (uint8_t)(strikes_[i] + 1);
      if (strikes_[i] >= maxStrikes_) {
        remove(i);
        continue;
      }
    }
    if (age_[i] < 0xFFFF) age_[i]++;
  }

  // 2) two points that converged onto the same spot: keep the older one
  for (int i = 0; i < n_; i++) {
    for (int j = i + 1; j < n_; j++) {
      const float dx = pts_[i].x - pts_[j].x, dy = pts_[i].y - pts_[j].y;
      if (dx * dx + dy * dy >= merge2_) continue;
      if (age_[j] > age_[i]) {                     // keep the older point in slot i
        pts_[i]     = pts_[j];
        ids_[i]     = ids_[j];
        age_[i]     = age_[j];
        strikes_[i] = strikes_[j];
      }
      remove(j);
      j--;
    }
  }
}

int Replenish::replenish(const Image& img) {
  const int sectors = sx_ * sy_;
  int have[MAX_SECTORS] = {0};                     // points per sector
  for (int i = 0; i < n_; i++) have[sectorOf(pts_[i], img)]++;

  int added = 0;
  for (int s = 0; s < sectors; s++) {
    const int need = quota(s) - have[s];
    if (need <= 0) continue;

    // detect only in this sector, away from every existing point (all sectors)
    const Rect r = sectorRect(s, img);
    const int got = gftt_.detect(img, found_, &r, pts_, n_, need);
    if (got == 0) continue;

    // budget full: drop the youngest points of sectors that are over their share
    while (n_ + got > maxPoints_) {
      int victim = -1;
      for (int i = 0; i < n_; i++) {
        const int t = sectorOf(pts_[i], img);
        if (have[t] > quota(t) && (victim < 0 || age_[i] < age_[victim])) victim = i;
      }
      if (victim < 0) break;                       // nobody over their share: keep the budget
      have[sectorOf(pts_[victim], img)]--;
      remove(victim);
    }

    const int take = (got < maxPoints_ - n_) ? got : maxPoints_ - n_;
    for (int k = 0; k < take; k++) {
      pts_[n_]     = found_[k];
      ids_[n_]     = nextId_++;
      age_[n_]     = 0;
      strikes_[n_] = 0;
      n_++;
    }
    have[s] += take;
    added += take;
  }
  return added;
}

void Replenish::remove(int i) {
  for (int k = i; k < n_ - 1; k++) {
    pts_[k]     = pts_[k + 1];
    ids_[k]     = ids_[k + 1];
    age_[k]     = age_[k + 1];
    strikes_[k] = strikes_[k + 1];
  }
  n_--;
}

int Replenish::quota(int sector) const {
  // budget split evenly; the first (budget % sectors) sectors get one extra
  const int sectors = sx_ * sy_;
  return maxPoints_ / sectors + (sector < maxPoints_ % sectors ? 1 : 0);
}

int Replenish::sectorOf(const Point2f& p, const Image& img) const {
  // same split as sectorRect(): the last column / row also takes the leftover pixels
  int cx = (int)p.x / (img.width / sx_);
  int cy = (int)p.y / (img.height / sy_);
  if (cx < 0) cx = 0;
  if (cy < 0) cy = 0;
  if (cx > sx_ - 1) cx = sx_ - 1;
  if (cy > sy_ - 1) cy = sy_ - 1;
  return cy * sx_ + cx;
}

Rect Replenish::sectorRect(int sector, const Image& img) const {
  const int sw = img.width / sx_, sh = img.height / sy_;
  const int cx = sector % sx_, cy = sector / sx_;
  Rect r;
  r.x = cx * sw;
  r.y = cy * sh;
  r.w = (cx == sx_ - 1) ? img.width - r.x : sw;
  r.h = (cy == sy_ - 1) ? img.height - r.y : sh;
  return r;
}

}  // namespace ccv
