// GC2145 exposure maths and register values (GC2145 CSP datasheet v1.0, 2013-12-01). Header
// only, no hardware: CameraReader writes / reads the registers, this decides what they mean.
//
// Timing (datasheet 7.1.1). The sensor is a rolling shutter: rows are read one after another,
// each exposed for `exposure` rows of time just before it is read.
//   row time   = Hb + Sh_delay + win_width + 4 pixel clocks      (P0:0x05/06, 0x11/12, 0x0f/10)
//   frame time = VB + win_height rows                             (P0:0x07/08, 0x0d/0e)
//                (longer if the exposure is longer: then the exposure sets the frame rate)
// In the 320x240 mode the library reads a 976 x 728 window, subsampled by 3; with VB = 50 that
// is 778 rows per frame. The pixel clock is not documented, so the row time is measured:
// frame period / rows (46.5 ms / 778 = ~60 us on the board).
//
// Exposure registers:
//   P0:0x03/04  exposure in rows (13 bits); written by the AEC while it runs, by us when it is off
//   P0:0xb6     bit 0: AEC on
//   P0:0xb0     global gain (4.4: 0x40 = 4.0);  P0:0xb1 / 0xb2: pre / post gain (set by the AEC)
//   P1:0x13     AEC target brightness;          P1:0x14: current frame brightness (read only)
//   P1:0x20     AEC max post digital gain (datasheet default 0xc0, the library sets 0x40)
//   P1:0x25/26  AEC anti-flicker step, rows: the AEC uses whole steps when it can
//   P1:0x27..34 AEC exposure levels 1..7, rows; P1:0x3c [6:5] which level is the maximum
// To cap the AEC every level and the flicker step are set to the cap: whatever level is the
// maximum, the exposure cannot go past it, and the AEC makes up the light with gain.

#pragma once

#include <stdint.h>

namespace vision {
namespace exposure {

struct Reg {
  uint8_t page, reg, value;
};

inline uint16_t usToRows(uint32_t us, float rowUs) {
  const float r = us / rowUs + 0.5f;
  return r < 1.0f ? 1 : (r > 8191.0f ? 8191 : (uint16_t)r);
}

inline float rowTimeUs(uint32_t framePeriodUs, uint16_t vb, uint16_t winHeight) {
  return (float)framePeriodUs / (vb + winHeight);
}

// How long before the end of the readout the middle of the image was seen: from the centre
// row's readout to the last row's (rolling shutter), plus half the exposure (the centre of
// the exposure ends at the readout).
inline uint32_t delayUs(uint16_t rowsCentreToEnd, uint16_t exposureRows, float rowUs) {
  return (uint32_t)((rowsCentreToEnd + exposureRows * 0.5f) * rowUs + 0.5f);
}

// Registers for "AEC on, never longer than capRows". Returns how many were written into out
// (needs 21). Ends on page 0.
inline int capRegs(uint16_t capRows, uint8_t maxPostGain, uint8_t expMinHigh, Reg* out) {
  int n = 0;
  const uint8_t hi = (capRows >> 8) & 0x1f, lo = capRows & 0xff;
  out[n++] = {0, 0xfe, 0x01};                  // page 1 (the page register itself, written on any page)
  out[n++] = {1, 0x25, hi};                    // anti-flicker step = the cap
  out[n++] = {1, 0x26, lo};
  for (uint8_t r = 0x27; r <= 0x34; r += 2) {  // exposure levels 1..7 = the cap
    out[n++] = {1, r, hi};
    out[n++] = {1, (uint8_t)(r + 1), lo};
  }
  out[n++] = {1, 0x3c, (uint8_t)(expMinHigh & 0x1f)};   // max level setting 0, keep exp min [12:8]
  out[n++] = {1, 0x20, maxPostGain};
  out[n++] = {1, 0xfe, 0x00};                  // back to page 0
  out[n++] = {0, 0xb6, 0x01};                  // AEC on
  return n;
}

// Registers for a fixed exposure (AEC off). Ends on page 0. (needs 5)
inline int manualRegs(uint16_t rows, uint8_t globalGain, Reg* out) {
  int n = 0;
  out[n++] = {0, 0xfe, 0x00};
  out[n++] = {0, 0xb6, 0x00};                  // AEC off first, or it overwrites the exposure
  out[n++] = {0, 0x03, (uint8_t)((rows >> 8) & 0x1f)};
  out[n++] = {0, 0x04, (uint8_t)(rows & 0xff)};
  out[n++] = {0, 0xb0, globalGain};
  return n;
}

}  // namespace exposure
}  // namespace vision
