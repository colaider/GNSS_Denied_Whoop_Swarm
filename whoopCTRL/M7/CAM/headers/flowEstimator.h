#pragma once

#include <stdint.h>

#include "../../../calibartion/camera_calib.h"
#include "../closedCV/headers/GFTT.h"
#include "../closedCV/headers/RANSAC.h"
#include "../closedCV/headers/pyramidLK.h"
#include "../closedCV/headers/replenish.h"

namespace vision {

struct FlowConfig {
  int   pyramidLevels = 4;       // 4 levels: up to ~15 px of motion per frame
  int   window        = 9;       // LK window, pixels (odd)
  float quality       = 0.005f;   // GFTT: keep corners >= quality * best corner
  float minDistance   = 8.0f;    // GFTT: spacing between points, pixels
  int   maxPoints     = 32;      // budget: points alive in the whole image (tracking cost grows with it;
                                 // was 100, which Replenish caps at 64: ~12 fps while moving)
  int   lkIterations  = 5;       // LK iterations per pyramid level at most (was 10)
  int   ransacIters   = 20;      // RANSAC random tries
  float inlierPx      = 0.7f;    // RANSAC: largest error that still counts as inlier
  int   minInliers    = 6;       // fewer -> the result is not valid
  float refillBelow   = 0.5f;    // search a sector for new points when below this share of its quota
  int   searchesPerFrame = 1;    // most sectors searched per frame (the search is the slow part)
};

struct FlowTiming {              // where the last update() spent its time, microseconds
  uint32_t pyramidUs   = 0;      // building the image pyramid (lk.newFrame)
  uint32_t trackUs     = 0;      // tracking the points (LK, forward + backward)
  uint32_t ransacUs    = 0;      // RANSAC + filling the results
  uint32_t replenishUs = 0;      // dropping lost points + searching new ones (GFTT)
  uint8_t  searches    = 0;      // sectors searched this frame
};

struct FlowResult {              // motion between the previous frame (t0) and this one (t1)
  bool     valid    = false;     // false: first frame, or too few points agree
  uint32_t seq      = 0;         // frame number of this frame
  uint32_t t0Us     = 0;         // time of the previous frame, microseconds
  uint32_t t1Us     = 0;         // time of this frame
  float    tx = 0.0f, ty = 0.0f; // image motion at the optical axis, pixels
  float    angle    = 0.0f;      // rotation about the optical axis, radians
  float    scale    = 1.0f;      // image scale change (> 1: moving closer to the floor)
  uint16_t nTracked = 0;         // points LK followed (passed forward-backward)
  uint16_t nInliers = 0;         // points that agree with the motion
  float    rmsPx    = 0.0f;      // error of the inliers, pixels
  float    fx = 0.0f, fy = 0.0f; // focal lengths of this image, pixels (pixels -> angle)
  float    cx = 0.0f, cy = 0.0f; // optical centre of this image, pixels
};

struct PointFlow {               // one tracked point of the last update()
  uint16_t id;                   // stays the same while the point is tracked
  uint16_t age;                  // frames it has been tracked
  float    x, y;                 // position in the newest frame, pixels
  float    dx, dy;               // motion since the previous frame, pixels
  uint8_t  inlier;               // 1 = agrees with the frame motion
};

class FlowEstimator {
 public:
  explicit FlowEstimator(const FlowConfig& cfg = FlowConfig());

  // Camera intrinsics of the images you will give (CameraReader::cropIntrinsics()).
  void begin(const calib::CameraIntrinsics& k);

  // Processes the newest frame. Returns getResult().valid.
  bool update(const ccv::Image& img, uint32_t seq, uint32_t tUs);

  const FlowResult& getResult() const { return result_; }
  const FlowTiming& getTiming() const { return timing_; }
  const PointFlow*  getPoints(int& count) const {   // points tracked in the last update()
    count = nOut_;
    return out_;
  }
  void reset();                                     // forget points + previous frame

 private:
  static constexpr int MAXP = ccv::Replenish::MAX_POINTS;

  FlowConfig cfg_;
  float cx_ = 0.0f, cy_ = 0.0f, fx_ = 0.0f, fy_ = 0.0f;

  ccv::GFTT      gftt_;          // declared before rep_: rep_ keeps a reference to it
  ccv::PyramidLK lk_;
  ccv::RANSAC    ransac_;
  ccv::Replenish rep_;

  bool       havePrev_ = false;
  uint32_t   prevUs_   = 0;
  FlowResult result_;
  FlowTiming timing_;
  PointFlow  out_[MAXP];
  int        nOut_ = 0;

  // scratch for one update() (members, not locals: keeps the thread stack small)
  ccv::Point2f cur_[MAXP];       // tracked positions, in Replenish's order
  ccv::Point2f p_[MAXP], q_[MAXP];   // tracked pairs relative to the optical centre
  uint8_t      status_[MAXP], inlier_[MAXP], mask_[MAXP];
  int          idx_[MAXP];       // pair k -> point index
};

}  // namespace vision
