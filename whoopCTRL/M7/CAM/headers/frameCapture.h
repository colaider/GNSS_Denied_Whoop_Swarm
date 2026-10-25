// FrameCapture - camera frames into two buffers in turn, so the next frame is read in (DMA)
// while the current one is processed.
//
// Why: the camera library's grabFrame() is a blocking snapshot. It arms the capture, waits for
// the next frame and returns at its end; the sensor then starts the following frame only a few
// ms later (vertical blanking, ~7 ms here). Anything longer than that between two grabs misses
// a frame: 21 fps becomes 11 fps as soon as tracking takes >~7 ms (more than ~20 points).
// Here the next capture is armed right after a frame ends, before processing starts, so
// processing may take almost a whole frame period (~46 ms) without losing frames.
//
//   vision::FrameCapture cap;
//   cap.begin(bufA, bufB, bytes);              // after the camera library is set up
//   uint32_t tEnd; bool exact;
//   const uint8_t* raw = cap.next(500000, tEnd, exact);   // waits for the frame being read in,
//                                              // arms the next one, returns the finished one
//   ... process raw (valid until the next call) ...
//
// The hardware side is the dcmi:: functions: DCMI snapshot + DMA on the board (dcmiCapture.cpp),
// a simulated sensor on the PC. No heap memory, nothing to free.

#pragma once

#include <stdint.h>

namespace vision {

namespace dcmi {
void     start(uint8_t* buf, uint32_t bytes);  // arm: the next whole frame goes into buf (DMA)
bool     busy();                               // the armed frame is not complete yet
void     stop(uint8_t* buf, uint32_t bytes);   // after it (or to abort): DMA off, D-cache invalidated over buf
uint32_t frameEnds();                          // frame-end interrupts so far
uint32_t frameEndUs();                         // shared clock at the last one (end of readout)
}  // namespace dcmi

class FrameCapture {
 public:
  // Two buffers of `bytes` each (32-byte aligned, a whole number of cache lines).
  void begin(uint8_t* a, uint8_t* b, uint32_t bytes);

  // Waits (up to timeoutUs) for the frame being read in, arms the next one into the other
  // buffer at once, and returns the finished one (valid until the next call). tEndUs: end of
  // its readout; exact = false if it is only when the end was noticed (no frame-end interrupt
  // and the frame was already complete). nullptr on timeout (dropped, re-armed next call).
  const uint8_t* next(uint32_t timeoutUs, uint32_t& tEndUs, bool& exact);

  uint32_t lastWaitUs() const { return waitUs_; }   // time next() waited for the frame

 private:
  uint8_t* buf_[2] = {nullptr, nullptr};
  uint32_t bytes_  = 0;
  int      cur_    = 0;                         // buffer being read in
  bool     armed_  = false;
  uint32_t ends0_  = 0;                         // frameEnds() when it was armed
  uint32_t waitUs_ = 0;

  void arm();
};

}  // namespace vision
