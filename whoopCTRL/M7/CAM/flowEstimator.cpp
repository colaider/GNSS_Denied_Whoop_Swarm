#include "headers/flowEstimator.h"

namespace vision {

FlowEstimator::FlowEstimator(const FlowConfig& cfg)
    : cfg_(cfg),
      gftt_(cfg.maxPoints, cfg.quality, cfg.minDistance),
      lk_(cfg.pyramidLevels, cfg.window),
      ransac_(cfg.ransacIters, cfg.inlierPx, cfg.minInliers),
      rep_(gftt_, cfg.maxPoints) {}

void FlowEstimator::begin(const calib::CameraIntrinsics& k) {
  cx_ = k.cx;
  cy_ = k.cy;
  fx_ = k.fx;
  fy_ = k.fy;
  reset();
}

void FlowEstimator::reset() {
  rep_.clear();
  havePrev_ = false;
  nOut_     = 0;
  result_   = FlowResult();
}

bool FlowEstimator::update(const ccv::Image& img, uint32_t seq, uint32_t tUs) {
  nOut_   = 0;
  result_ = FlowResult();
  result_.seq  = seq;
  result_.t0Us = prevUs_;
  result_.t1Us = tUs;
  result_.fx   = fx_;
  result_.fy   = fy_;
  result_.cx   = cx_;
  result_.cy   = cy_;
  if (!lk_.newFrame(img)) return false;            // image too large for the tracker

  const int n = rep_.count();
  if (havePrev_ && n > 0) {
    const ccv::Point2f* prev = rep_.points();

    // 1) track every kept point into this frame (forward-backward check inside)
    lk_.track(prev, cur_, status_, n);

    // 2) RANSAC on the tracked ones, relative to the optical centre
    int m = 0;
    for (int i = 0; i < n; i++) {
      inlier_[i] = 0;
      if (!status_[i]) continue;
      p_[m]   = {prev[i].x - cx_, prev[i].y - cy_};
      q_[m]   = {cur_[i].x - cx_, cur_[i].y - cy_};
      idx_[m] = i;
      m++;
    }
    ccv::Similarity sim;
    const bool ok = ransac_.estimate(p_, q_, m, sim, mask_);
    for (int k = 0; k < m; k++) inlier_[idx_[k]] = mask_[k];

    // 3) results: the frame motion and every tracked point
    result_.valid    = ok;
    result_.tx       = sim.tx;
    result_.ty       = sim.ty;
    result_.angle    = sim.angle;
    result_.scale    = sim.scale;
    result_.nTracked = (uint16_t)m;
    result_.nInliers = (uint16_t)sim.nInliers;
    result_.rmsPx    = sim.rms;
    for (int k = 0; k < m; k++) {
      const int i = idx_[k];
      PointFlow& o = out_[nOut_++];
      o.id     = rep_.id(i);
      o.age    = (uint16_t)(rep_.age(i) + 1);
      o.x      = cur_[i].x;
      o.y      = cur_[i].y;
      o.dx     = cur_[i].x - prev[i].x;
      o.dy     = cur_[i].y - prev[i].y;
      o.inlier = inlier_[i];
    }

    // 4) keep the survivors. Strikes only when RANSAC found a motion: with too few
    //    points nobody can be called an outlier.
    rep_.update(cur_, status_, ok ? inlier_ : nullptr);
  }

  // 5) top up the sectors for the next frame
  rep_.replenish(img);
  havePrev_ = true;
  prevUs_   = tUs;
  return result_.valid;
}

}  // namespace vision
