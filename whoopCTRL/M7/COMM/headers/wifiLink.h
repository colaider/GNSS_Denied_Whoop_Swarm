// WifiLink - UDP over the Nicla's WiFi, to the PC.
//
// - begin(): ap = true makes the Nicla's own network (the PC joins it, the Nicla is 192.168.3.1);
//   ap = false joins an existing network (waits up to 10 s). Then listens on `port`.
// - Packets go to whoever sent the last datagram (the PC's address is learnt from its
//   commands): nothing goes out before the PC has sent something.
// - One Communication packet = one datagram (collected between lock() and unlock()).
// Needs the WiFi firmware on the Nicla (examples > STM32H747_System > WiFiFirmwareUpdater, once).

#pragma once

#include "../../../shared/communication/headers/link.h"

namespace comm {

class WifiLink : public Link {
 public:
  static constexpr size_t MAX_DATAGRAM = 508;    // WiFiUDP's send buffer (WIFI_UDP_BUFFER_SIZE): longer ones went out cut, CRC failed on the PC

  WifiLink(const char* ssid, const char* pass, bool ap, uint16_t port)
      : ssid_(ssid), pass_(pass), ap_(ap), port_(port) {}

  bool        begin() override;
  size_t      write(const uint8_t* data, size_t len) override;
  int         read() override;
  const char* name() const override { return "wifi"; }
  void        lock() override;                    // starts collecting a datagram
  void        unlock() override;                  // sends it to the PC

  bool        hasPeer() const { return havePeer_; }   // the PC has sent something
  const char* ip() const { return ip_; }              // the Nicla's address, text

 private:
  const char* ssid_;
  const char* pass_;
  bool        ap_;
  uint16_t    port_;
  bool        havePeer_ = false;
  char        ip_[16] = "0.0.0.0";

  uint8_t     tx_[MAX_DATAGRAM];
  size_t      txN_ = 0;
  bool        txOver_ = false;                    // packet too big for one datagram: dropped
  uint8_t     rx_[MAX_DATAGRAM];
  size_t      rxN_ = 0, rxPos_ = 0;
};

}  // namespace comm
