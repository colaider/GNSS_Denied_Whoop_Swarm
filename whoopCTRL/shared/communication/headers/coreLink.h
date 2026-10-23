// CoreLink - the link between the M7 and the M4 (Arduino RPC library, raw channel).
// Used the same way on both cores: comm::addLink(core); comms.begin("core");
//
// - begin() on the M7 also STARTS the M4 and waits up to 5 s for it to answer
//   (false if the M4 has no program). On the M4 it connects to the M7.
// - Sending: the bytes of one packet are collected while Communication holds the lock
//   and sent at unlock(), so a packet is usually one RPC message (split above CHUNK).
// - Receiving: every RPC message is copied into a 2 KB buffer as it arrives; read() takes
//   bytes from there and never waits.
// - Do not use RPC.print()/RPC.write() elsewhere: that would mix text into the packets.
//
// One CoreLink per core (the RPC library has one raw channel). No heap, nothing to free.

#pragma once

#include "link.h"

namespace comm {

class CoreLink : public Link {
 public:
  static constexpr size_t CHUNK   = 256;    // largest RPC message we send (the limit is ~496)
  static constexpr size_t RX_SIZE = 2048;   // receive buffer, bytes

  bool        begin() override;
  size_t      write(const uint8_t* data, size_t len) override;
  int         read() override;
  const char* name() const override { return "core"; }
  void        lock() override;              // starts collecting a packet
  void        unlock() override;            // sends what was collected

  uint32_t    rxOverflows() const;          // bytes dropped because the receive buffer was full
};

}  // namespace comm
