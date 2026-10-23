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

  visualVelocityVector estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::ToFReading& tof, const msg::FlowHeader& flow);

  vw linReg(const float* x, const float* y, int n);
};
