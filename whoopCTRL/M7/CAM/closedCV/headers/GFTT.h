// GFTT - Shi-Tomasi "good features to track".
// Score of a pixel = smallest eigenvalue of G = sum [Ix^2 IxIy; IxIy Iy^2] over a 5x5 window
// (central-difference gradients). Points come out strongest first, at least `minDistance`
// apart and away from points you already track.
// No heap memory, nothing to free. Holds ~8 KB: make it a global, not a local variable.

#pragma once

#include <stdint.h>

#include "cvTypes.h"

namespace ccv {

class GFTT {
 public:
  static constexpr int MAX_CANDIDATES = 512;  
  static constexpr int MAX_ROW        = 164;   

  GFTT(int maxFeatures, float quality, float minDistance = 8.0f, float minScore = 500.0f, int border = 10);

  int detect(const Image& img, Point2f* out, const Rect* roi = nullptr, const Point2f* existing = nullptr, int nExisting = 0, int maxOut = 0);

 private:
  struct Candidate {
    int16_t x, y;
    float   score;
  };

  static float score(const Image& img, int x, int y);   // smallest eigenvalue at (x, y)
  void addCandidate(int x, int y, float s);

  int   maxFeatures_;
  float quality_;
  float minDist2_;                 // minDistance squared
  float minScore_;
  int   border_;

  Candidate cand_[MAX_CANDIDATES];
  int       nCand_ = 0;
  float     rows_[3][MAX_ROW];     // scores of 3 consecutive rows, for the local-maximum test
};

}  // namespace ccv
