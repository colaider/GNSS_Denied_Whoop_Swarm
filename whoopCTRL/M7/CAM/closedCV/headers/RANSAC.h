#pragma once

#include <stdint.h>

#include "cvTypes.h"

namespace ccv {

struct Similarity {
  float a = 1.0f, b = 0.0f;        // q = [a -b; b a] * p + t
  float tx = 0.0f, ty = 0.0f;      // pixels
  float scale = 1.0f;
  float angle = 0.0f;              // radians
  int   nInliers = 0;
  float rms = 0.0f;                // inlier error, pixels
};

class RANSAC {
 public:
  // iterations:    random 2-point tries (about 20 handles 50% bad points)
  // threshold:     largest error in pixels that still counts as an inlier
  // minInliers:    fewer inliers -> estimate() returns false
  // minSampleDist: the two points of a try must be at least this far apart, pixels
  explicit RANSAC(int iterations, float threshold = 0.7f, int minInliers = 6,
                  float minSampleDist = 10.0f);

  // Fits q ~ model(p) for n point pairs. inlier (optional, n entries) gets 1 for inliers,
  // 0 for outliers. Returns false if there are too few points or too few inliers.
  bool estimate(const Point2f* p, const Point2f* q, int n, Similarity& out,
                uint8_t* inlier = nullptr);

 private:
  int  countInliers(const Point2f* p, const Point2f* q, int n, const Similarity& m,
                    float* sumErr2) const;
  bool refit(const Point2f* p, const Point2f* q, int n, Similarity& m) const;  // LS on inliers
  uint32_t random();                                                           // xorshift32

  int      iterations_;
  float    thr2_;                  // threshold squared
  int      minInliers_;
  float    minDist2_;              // minSampleDist squared
  uint32_t rng_ = 2463534242u;
};

}  // namespace ccv
