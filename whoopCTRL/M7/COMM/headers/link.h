// Link - a byte pipe (USB serial, WiFi, the M4 core...).
// It only moves bytes; packets (framing, checksum) are done by Communication on top,
// so every link speaks the same protocol.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace comm {

class Link {
 public:
  virtual ~Link() {}

  virtual bool        begin() = 0;                                  // open the link
  virtual size_t      write(const uint8_t* data, size_t len) = 0;   // send bytes, returns bytes sent
  virtual int         read() = 0;                                   // next received byte, -1 if none (never waits)
  virtual const char* name() const = 0;                             // "usb", "wifi", ...
};

}  // namespace comm
