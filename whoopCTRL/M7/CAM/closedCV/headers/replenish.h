// Replenish - the list of tracked points, kept alive across frames.
//
// Points are reused: update() only moves surviving points and removes lost ones, and
// replenish() only adds new points. A point keeps its id for as long as it can be tracked.
//
// The image is split into sectors (2x2 by default) and the point budget is shared between
// them, so the points stay spread over the image. New points are searched only in sectors
// that are below their share, at least GFTT's minDistance from every existing point.
// If the budget is full, the youngest points of over-full sectors make room for them.
//
// Every frame:
//   lk.newFrame(img);
//   lk.track(rep.points(), cur, status, rep.count());
//   ... RANSAC on the tracked points, inlier[i] per track ...
//   rep.update(cur, status, inlier);      // move / drop
//   rep.replenish(img);                   // top up the sectors
//
// No heap memory, nothing to free. `gftt` is used, not owned: it must stay alive.

#pragma once

#include <stdint.h>

#include "GFTT.h"
#include "cvTypes.h"

namespace ccv {

class Replenish {
 public:
  static constexpr int MAX_POINTS  = 64;
  static constexpr int MAX_SECTORS = 16;          // up to a 4 x 4 grid

  // gftt:          detector for new points (its minDistance keeps new points apart)
  // maxPoints:     budget, the most points alive in the whole image (<= MAX_POINTS)
  // sectorsX/Y:    sector grid (2 x 2 = 4 sectors), each at most 4
  // maxStrikes:    a point that is a RANSAC outlier this many frames in a row is dropped
  // mergeDistance: if two points come closer than this (pixels), the younger one is dropped
  Replenish(GFTT& gftt, int maxPoints, int sectorsX = 2, int sectorsY = 2, int maxStrikes = 2,
            float mergeDistance = 3.0f);

  // After tracking: newPts / status / inlier have count() entries, in the order of points().
  // status 0 -> point lost, removed. inlier (optional) 0 -> one strike.
  void update(const Point2f* newPts, const uint8_t* status, const uint8_t* inlier = nullptr);

  // Adds new points from `img` into sectors below their share. Returns how many were added.
  int replenish(const Image& img);

  int            count()     const { return n_; }
  const Point2f* points()    const { return pts_; }   // positions, for PyramidLK::track
  uint16_t       id(int i)   const { return ids_[i]; }
  uint16_t       age(int i)  const { return age_[i]; } // frames this point has been tracked
  void           clear()           { n_ = 0; }

 private:
  void remove(int i);               // removes point i, keeps the order of the others
  int  quota(int sector) const;     // this sector's share of the budget
  int  sectorOf(const Point2f& p, const Image& img) const;
  Rect sectorRect(int sector, const Image& img) const;

  GFTT& gftt_;
  int   maxPoints_, sx_, sy_, maxStrikes_;
  float merge2_;                    // mergeDistance squared

  Point2f  pts_[MAX_POINTS];
  uint16_t ids_[MAX_POINTS];
  uint16_t age_[MAX_POINTS];
  uint8_t  strikes_[MAX_POINTS];
  int      n_      = 0;
  uint16_t nextId_ = 0;

  Point2f  found_[MAX_POINTS];      // scratch for GFTT results
};

}  // namespace ccv
