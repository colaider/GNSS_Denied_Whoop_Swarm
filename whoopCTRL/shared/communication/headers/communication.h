// Communication - packets over a link chosen by name. Used by both cores.
//
//   comm::UsbLink usb;                         // each sketch creates the links it has
//   comm::addLink(usb);                        // ...and registers them once
//   comm::Communication comms;
//   comms.begin("usb");                        // pick a registered link by its name()
//   comms.send(msg::PKT_LOG, "hello", 5);
//   comm::Packet p;
//   while (comms.receive(p)) { ... }           // never waits
//
// One Communication per link (it keeps that link's receive state). Packet types and
// layouts are in shared/headers/messages.h.
//
// Packet on the wire (little-endian):
//   0xA5 0x5A | type u8 | len u32 | payload[len] | crc16 u16
//   crc16 = CRC-16/CCITT-FALSE over type, len and payload

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "link.h"

namespace comm {

constexpr int      MAX_LINKS      = 4;     // links one sketch can register
constexpr uint32_t MAX_RX_PAYLOAD = 256;   // received packets are small (commands, results)

// Makes a link available to Communication::begin() under its name(). False if full.
bool addLink(Link& link);

struct Packet {
  uint8_t  type = 0;
  uint32_t len  = 0;
  uint8_t  data[MAX_RX_PAYLOAD];
};

class Communication {
 public:
  // Picks the registered link with this name and opens it. False if unknown or it fails.
  bool begin(const char* linkName);

  // One packet. The two-part form sends a + b as one payload without copying
  // (e.g. a small header + a big image).
  bool send(uint8_t type, const void* data, uint32_t len);
  bool send(uint8_t type, const void* a, uint32_t na, const void* b, uint32_t nb);
  bool log(const char* text);                     // PKT_LOG with a text

  // Reads what has arrived; true when a complete packet with a valid checksum is in `out`.
  // Never waits. Call it in a loop until it returns false.
  bool receive(Packet& out);

  const char* linkName() const { return link_ ? link_->name() : "none"; }
  uint32_t    rxErrors() const { return rxErrors_; }   // packets dropped for a bad checksum

 private:
  enum RxState : uint8_t { RX_MAGIC1, RX_MAGIC2, RX_HEADER, RX_PAYLOAD, RX_CRC };

  Link* link_ = nullptr;

  // receive state machine
  RxState  rxState_ = RX_MAGIC1;
  uint8_t  rxHead_[5];                            // type + 4 length bytes
  uint8_t  rxCrc_[2];
  uint8_t  rxBuf_[MAX_RX_PAYLOAD];
  uint32_t rxLen_    = 0;
  uint32_t rxPos_    = 0;
  uint32_t rxErrors_ = 0;
};

}  // namespace comm
