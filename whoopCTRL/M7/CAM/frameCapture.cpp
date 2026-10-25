#include "headers/frameCapture.h"

#include "../../shared/headers/sharedClock.h"

namespace vision {

void FrameCapture::begin(uint8_t* a, uint8_t* b, uint32_t bytes) {
  buf_[0] = a;
  buf_[1] = b;
  bytes_  = bytes;
  cur_    = 0;
  armed_  = false;
}

void FrameCapture::arm() {
  ends0_ = dcmi::frameEnds();
  dcmi::start(buf_[cur_], bytes_);
  armed_ = true;
}

const uint8_t* FrameCapture::next(uint32_t timeoutUs, uint32_t& tEndUs, bool& exact) {
  if (!armed_) arm();                          // first call, or after a timeout

  const uint32_t t0 = shared::nowUs();
  bool waited = false;
  while (dcmi::busy()) {
    waited = true;
    if ((uint32_t)(shared::nowUs() - t0) > timeoutUs) {
      dcmi::stop(buf_[cur_], bytes_);          // no frame: drop it, arm again next call
      armed_ = false;
      waitUs_ = shared::nowUs() - t0;
      return nullptr;
    }
  }
  const uint32_t seen = shared::nowUs();
  waitUs_ = seen - t0;
  dcmi::stop(buf_[cur_], bytes_);

  // End of readout: the frame-end interrupt's time if it came for this frame. Without it, the
  // moment the end was seen is exact only if we were waiting for it; if the frame was already
  // complete (processing took longer than a frame), it is late by up to a frame period.
  const bool irq = dcmi::frameEnds() != ends0_;
  exact  = irq || waited;
  tEndUs = irq ? dcmi::frameEndUs() : seen;

  uint8_t* done = buf_[cur_];
  cur_ ^= 1;
  arm();                                       // the next frame starts within the blanking: catch it now
  return done;
}

}  // namespace vision
