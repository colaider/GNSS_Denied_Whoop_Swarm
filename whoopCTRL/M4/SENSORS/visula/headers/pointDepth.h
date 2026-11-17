// PointDepth - the depth (and so the 3D position, the height) of every tracked camera point, from
// how far it moves while the camera moves (parallax). Watching only: nothing here goes into the
// filters, the velocity or the control.
//
// Per frame pair a still point at depth Z moves in the image (gyro rotation removed, in x / f units)
//   m = -(T + p dZ) / Z
// T: the camera's move across the view = the camera velocity Sensors measured x pair time (that
// velocity takes the ToF range for every point: floor points come out at the ToF range, a box top
// nearer), p: the point's (x, y) / f, dZ: the distance change over the pair (ToF trend).
// One pair moves a point only ~0.2 px at hover (tracking noise 0.1-0.2 px), so each point keeps 1/Z
// with its variance (a 1-state Kalman filter) and every pair adds a little. A new point starts at the
// ToF range +-50 %: without motion (no parallax) it stays there.
//
//   sensors::PointDepth depth;                           // global
//   sens.update(&flow, pts, n);                          // first: the camera velocity of this pair
//   depth.update(sens, flow, pts, n);
//   for (int i = 0; i < depth.count(); i++) depth.point(i).world ...
//
// No heap memory, nothing to free.

#pragma once

#include <stdint.h>

#include "../../headers/SENSORS.h"

namespace sensors {

struct PointDepthConfig {
  float pxNoise = 0.15f;         // tracking noise per frame pair, px (RANSAC rms 0.1-0.2)
  float initRel = 0.5f;          // a new point: ToF range +- this share
  float gate    = 4.0f;          // a pair further off than this many std is not taken (a bad track)
  float minZ    = 0.02f;         // depth kept within, m
  float maxZ    = 3.0f;
};

class PointDepth {
 public:
  static constexpr int MAXP = 96;               // = the most points a flow packet carries (msg::FLOW_MAX_POINTS)

  struct Point {
    uint16_t id;
    float    x, y;               // image, px
    float    depth, std;         // along the view, m, and its 1 std
    Vec3     world;              // start frame, m (z: height above the floor)
  };

  explicit PointDepth(const PointDepthConfig& cfg = PointDepthConfig()) : cfg_(cfg) {}

  // After Sensors::update() with the same flow packet. Points not in it are forgotten.
  void update(const Sensors& s, const msg::FlowHeader& f, const msg::PointMsg* pts, int n);

  int          count() const { return n_; }
  const Point& point(int i) const { return out_[i]; }

 private:
  struct Track { uint16_t id; float rho, P; };   // 1 / depth (1/m), its variance

  PointDepthConfig cfg_;
  Track tr_[MAXP];
  Point out_[MAXP];
  int   n_ = 0;
};

}  // namespace sensors
