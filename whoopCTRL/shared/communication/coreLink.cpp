#include "headers/coreLink.h"

#include <Arduino.h>
#include <RPC.h>
#include <string.h>

#include "mbed.h"

namespace comm {

// The RPC library has one raw channel and calls a plain function, so the state lives here
// (one CoreLink per core). Names start with 'core' because sketches include several .cpp files
// into one compile unit, where file-private names must not collide.

// receive: ring buffer, filled by the RPC thread, emptied by read()
static uint8_t           coreRxBuf[CoreLink::RX_SIZE];
static volatile uint32_t coreRxHead    = 0;     // next write position (RPC thread only)
static volatile uint32_t coreRxTail    = 0;     // next read position (reader only)
static volatile uint32_t coreOverflows = 0;
static uint32_t          coreRxLeft    = 0;     // bytes of the current packet still to come (RPC thread only)
static bool              coreRxKeep    = true;  // keep or drop the current packet

// send: bytes collected under the lock, sent as one RPC message per CHUNK
static rtos::Mutex coreTxMutex;                 // recursive: lock() may nest
static uint8_t     coreTxBuf[CoreLink::CHUNK];
static size_t      coreTxLen     = 0;
static int         coreLockDepth = 0;

// Called by the RPC thread with one whole message from the other core.
// A packet bigger than CHUNK arrives as several messages, and a new packet always starts a
// new message. Keep or drop is decided ONCE, at the packet's first message, for the whole
// packet (its length is in the header), so the buffer only ever holds complete packets
// (keeping half of one would corrupt the packet after it too).
static void onCoreRaw(const uint8_t* buf, size_t len) {
  const uint32_t head = coreRxHead;
  const uint32_t space = (coreRxTail + CoreLink::RX_SIZE - head - 1) % CoreLink::RX_SIZE;

  if (coreRxLeft == 0) {                                   // the first message of a packet
    uint32_t total = len;                                  // no packet header: just this message
    if (len >= 7 && buf[0] == 0xA5 && buf[1] == 0x5A) {    // A5 5A | type | len u32 (communication.cpp)
      const uint32_t payload = buf[3] | (uint32_t)buf[4] << 8 | (uint32_t)buf[5] << 16 | (uint32_t)buf[6] << 24;
      total = 7 + payload + 2;                             // header + payload + crc
    }
    coreRxKeep = total <= space;                           // room for the WHOLE packet now
    coreRxLeft = total;
  }
  coreRxLeft = coreRxLeft > len ? coreRxLeft - len : 0;
  if (!coreRxKeep || len > space) {                        // (len > space cannot happen once kept)
    coreRxKeep = false;
    coreOverflows += len;
    return;
  }
  for (size_t i = 0; i < len; i++) coreRxBuf[(head + i) % CoreLink::RX_SIZE] = buf[i];
  coreRxHead = (head + len) % CoreLink::RX_SIZE;  // publish only after all bytes are in
}

static void coreFlush() {
  if (coreTxLen == 0) return;
  RPC.write(coreTxBuf, coreTxLen, true);            // 3 arguments: true = raw channel (2 would be ambiguous)
  coreTxLen = 0;
}

bool CoreLink::begin() {
  RPC.attach(onCoreRaw);                        // before begin(): nothing lands in RPC's own buffer
  return RPC.begin() == 1;                  // M7: boots the M4, gives up after 5 s
}

size_t CoreLink::write(const uint8_t* data, size_t len) {
  lock();                                   // nested inside Communication::send's lock
  for (size_t i = 0; i < len;) {
    size_t n = CHUNK - coreTxLen;
    if (n > len - i) n = len - i;
    memcpy(coreTxBuf + coreTxLen, data + i, n);
    coreTxLen += n;
    i += n;
    if (coreTxLen == CHUNK) coreFlush();
  }
  unlock();                                 // sends now only if this was the outermost lock
  return len;
}

int CoreLink::read() {
  if (coreRxTail == coreRxHead) return -1;
  const uint8_t b = coreRxBuf[coreRxTail];
  coreRxTail = (coreRxTail + 1) % RX_SIZE;
  return b;
}

void CoreLink::lock() {
  coreTxMutex.lock();
  coreLockDepth++;
}

void CoreLink::unlock() {
  if (--coreLockDepth == 0) coreFlush();            // end of the packet: send it
  coreTxMutex.unlock();
}

uint32_t CoreLink::rxOverflows() const {
  return coreOverflows;
}

}  // namespace comm
