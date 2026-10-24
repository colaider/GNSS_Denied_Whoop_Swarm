#pragma once

#include <stdint.h>

#include "../../headers/IMU.h"
#include "../../headers/ToF.h"
#include "../../../../shared/headers/messages.h"

class visualVel {
 public:
  struct visualVelocityVector { float vx = 0.0f; float vy = 0.0f; float vz = 0.0f; float yawRate = 0.0f; };
  struct vw { float v = 0.0f; float w = 0.0f; };

  visualVel();

  void getOpticalFlow(const msg::FlowHeader& flow);

  // rot: camera rotation from flow.t0Us to flow.t1Us (gyro), radians, camera axes: x right, y down, z along the view
  // dZ: change of the distance to the floor over the same time (ToF), m; < 0 = getting closer
  visualVelocityVector estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::Vec3& rot, const sensors::ToFReading& tof, float dZ, const msg::FlowHeader& flow, const msg::PointMsg* pts, int n);

  vw linReg(const float* x, const float* y, int n);
};
