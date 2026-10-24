#include "headers/communication.h"

#include <string.h>

#include "../headers/messages.h"

namespace comm {

static_assert(msg::FLOW_MAX_BYTES <= MAX_RX_PAYLOAD, "a flow packet with all its points must fit a received Packet");

static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;

// links registered by the sketch, looked up by name in Communication::begin()
static Link* links[MAX_LINKS];
static int   nLinks = 0;

bool addLink(Link& link) {
  if (nLinks >= MAX_LINKS) return false;
  links[nLinks++] = &link;
  return true;
}

// CRC-16/CCITT-FALSE (poly 0x1021, start 0xFFFF) - same as binascii.crc_hqx(data, 0xFFFF) in Python
static uint16_t crc16(const uint8_t* d, uint32_t n, uint16_t crc) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

bool Communication::begin(const char* linkName) {
  link_ = nullptr;
  for (int i = 0; i < nLinks && !link_; i++)
    if (strcmp(links[i]->name(), linkName) == 0) link_ = links[i];
  if (!link_) return false;                        // not registered with addLink()
  return link_->begin();
}

bool Communication::send(uint8_t type, const void* data, uint32_t len) {
  return send(type, data, len, nullptr, 0);
}

bool Communication::send(uint8_t type, const void* a, uint32_t na, const void* b, uint32_t nb) {
  if (!link_) return false;
  const uint32_t len = na + nb;
  const uint8_t head[7] = {MAGIC0, MAGIC1, type, (uint8_t)len, (uint8_t)(len >> 8),
                           (uint8_t)(len >> 16), (uint8_t)(len >> 24)};
  uint16_t crc = crc16(head + 2, 5, 0xFFFF);
  if (na) crc = crc16((const uint8_t*)a, na, crc);
  if (nb) crc = crc16((const uint8_t*)b, nb, crc);
  const uint8_t tail[2] = {(uint8_t)crc, (uint8_t)(crc >> 8)};

  link_->lock();                                   // one whole packet, never mixed with another
  size_t sent = link_->write(head, sizeof(head));
  if (na) sent += link_->write((const uint8_t*)a, na);
  if (nb) sent += link_->write((const uint8_t*)b, nb);
  sent += link_->write(tail, sizeof(tail));
  link_->unlock();
  return sent == sizeof(head) + len + sizeof(tail);
}

bool Communication::log(const char* text) {
  return send(msg::PKT_LOG, text, strlen(text));
}

bool Communication::receive(Packet& out) {
  if (!link_) return false;
  int c;
  while ((c = link_->read()) >= 0) {
    const uint8_t b = (uint8_t)c;
    switch (rxState_) {
      case RX_MAGIC1:
        if (b == MAGIC0) rxState_ = RX_MAGIC2;
        break;

      case RX_MAGIC2:
        rxPos_ = 0;
        rxState_ = (b == MAGIC1) ? RX_HEADER : (b == MAGIC0 ? RX_MAGIC2 : RX_MAGIC1);
        break;

      case RX_HEADER:                             // type + 4 length bytes
        rxHead_[rxPos_++] = b;
        if (rxPos_ == sizeof(rxHead_)) {
          rxLen_ = rxHead_[1] | (uint32_t)rxHead_[2] << 8 | (uint32_t)rxHead_[3] << 16 |
                   (uint32_t)rxHead_[4] << 24;
          rxPos_ = 0;
          if (rxLen_ > MAX_RX_PAYLOAD) {          // too long for us: not a valid packet
            rxErrors_++;
            rxState_ = RX_MAGIC1;
          } else {
            rxState_ = rxLen_ ? RX_PAYLOAD : RX_CRC;
          }
        }
        break;

      case RX_PAYLOAD:
        rxBuf_[rxPos_++] = b;
        if (rxPos_ == rxLen_) {
          rxPos_ = 0;
          rxState_ = RX_CRC;
        }
        break;

      case RX_CRC:
        rxCrc_[rxPos_++] = b;
        if (rxPos_ == sizeof(rxCrc_)) {
          rxState_ = RX_MAGIC1;
          uint16_t crc = crc16(rxHead_, sizeof(rxHead_), 0xFFFF);
          crc = crc16(rxBuf_, rxLen_, crc);
          if (crc == (uint16_t)(rxCrc_[0] | rxCrc_[1] << 8)) {
            out.type = rxHead_[0];
            out.len  = rxLen_;
            memcpy(out.data, rxBuf_, rxLen_);
            return true;                          // one packet per call; the rest stays queued
          }
          rxErrors_++;
        }
        break;
    }
  }
  return false;
}

}  // namespace comm
