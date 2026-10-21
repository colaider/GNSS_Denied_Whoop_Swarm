#pragma once
#include <stdint.h>
#include "cvTypes.h"

namespace ccv {

class PyramidLK {
 public:
  static constexpr int MAX_LEVELS = 4;
  static constexpr int MAX_WIN    = 15;                // largest window, pixels (odd)
  static constexpr int MAX_W      = 160;               // largest input image
  static constexpr int MAX_H      = 120;

  PyramidLK(int levels, int winSize, int maxIter = 10, float epsilon = 0.03f, float minEigen = 1.0f, float fbThreshold = 0.5f);

  // Gives the tracker a new frame. Builds its pyramid; the previous frame's pyramid is kept.
  // False if 
  bool newFrame(const Image& img);

  int track(const Point2f* prevPts, Point2f* curPts, uint8_t* status, int n, const Point2f* guess = nullptr);

  bool ready() const { return frames_ >= 2; }       // two frames given: tracking possible

 private:
  struct Level {
    uint8_t* data = nullptr;
    int w = 0, h = 0;
  };
  struct Pyramid {
    Level lv[MAX_LEVELS];
  };

  void buildPyramid(Pyramid& py, uint8_t* mem, const Image& img);
  bool trackPoint(const Pyramid& from, const Pyramid& to, Point2f p, Point2f guess, Point2f& q);
  static void sample(const Level& img, float x0, float y0, int size, float* out);  // bilinear

  int   levels_, win_, half_, maxIter_;
  float eps2_, minEigen_, fb2_;

  // storage for two pyramids: full image + 1/4 + 1/16 + 1/64 of it
  static constexpr int PYR_BYTES = MAX_W * MAX_H + MAX_W * MAX_H / 4 + MAX_W * MAX_H / 16 + MAX_W * MAX_H / 64;
  uint8_t store_[2][PYR_BYTES];
  Pyramid pyr_[2];
  int     cur_    = 0;             // index of the newest pyramid
  int     frames_ = 0;

  // scratch for one window: template (with a 1 px border for gradients) + its gradients
  float patch_[(MAX_WIN + 2) * (MAX_WIN + 2)];
  float gx_[MAX_WIN * MAX_WIN];
  float gy_[MAX_WIN * MAX_WIN];
  float jwin_[MAX_WIN * MAX_WIN];  // current frame's window, resampled each iteration
};

}  // namespace ccv
