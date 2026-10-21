// Small types shared by the closedCV algorithms.

#pragma once

#include <stdint.h>

namespace ccv {

struct Point2f {
  float x = 0.0f;
  float y = 0.0f;
};

struct Rect {                    // columns x .. x+w-1, rows y .. y+h-1
  int x = 0, y = 0, w = 0, h = 0;
};

struct Image {                   // read-only grey image: pixel (x, y) = data[y * width + x]
  const uint8_t* data = nullptr;
  int width  = 0;
  int height = 0;
};

}  // namespace ccv
