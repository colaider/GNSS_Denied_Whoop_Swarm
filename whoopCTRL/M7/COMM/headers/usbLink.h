// UsbLink - the USB serial port to the PC.

#pragma once

#include "../../../shared/communication/headers/link.h"

namespace comm {

class UsbLink : public Link {
 public:
  bool        begin() override;                                     // waits up to 3 s for the PC
  size_t      write(const uint8_t* data, size_t len) override;
  int         read() override;
  const char* name() const override { return "usb"; }
  void        lock() override;                                      // mutex: several threads may send
  void        unlock() override;
};

}  // namespace comm
