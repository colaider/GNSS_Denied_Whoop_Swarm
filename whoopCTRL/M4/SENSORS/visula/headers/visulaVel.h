#pragma once

#include <stdint.h>

#include "../../headers/IMU.h"
#include "../../headers/ToF.h"
#include "../../../../shared/headers/messages.h"
#include "../../../../calibartion/camera_calib.h"

class visualVel {
 public:
  struct visualVelocityVector { float vx = 0.0f; float vy = 0.0f; float vz = 0.0f; float yawRate = 0.0f; };
  struct vw { float v = 0.0f; float w = 0.0f; };

  static constexpr int MAXN = 96;   // points used (msg::FLOW_MAX_POINTS)

  visualVel();

  void getOpticalFlow(const msg::FlowHeader& flow);
  visualVelocityVector estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::Vec3& rot, const sensors::ToFReading& tof, float dZ, const msg::FlowHeader& flow, const msg::PointMsg* pts, int n);

  vw linReg(const float* x, const float* y, int n);
  static void undistort(float u, float v, const msg::FlowHeader& flow, float& x, float& y);
};
