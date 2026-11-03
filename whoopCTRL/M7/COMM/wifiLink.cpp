#include "headers/wifiLink.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "mbed.h"

namespace comm {

static WiFiUDP      wifiUdp;
static IPAddress    wifiPeerIp;
static uint16_t     wifiPeerPort = 0;
static rtos::Mutex  wifiTxMutex;

bool WifiLink::begin() {
  int st;
  if (ap_) {
    st = WiFi.beginAP(ssid_, pass_);
    if (st != WL_AP_LISTENING) return false;
  } else {
    st = WiFi.begin(ssid_, pass_);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(100);
    if (WiFi.status() != WL_CONNECTED) return false;
  }
  const IPAddress a = WiFi.localIP();
  snprintf(ip_, sizeof(ip_), "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
  return wifiUdp.begin(port_) == 1;
}

size_t WifiLink::write(const uint8_t* data, size_t len) {
  if (txN_ + len > MAX_DATAGRAM) { txOver_ = true; return 0; }
  memcpy(tx_ + txN_, data, len);
  txN_ += len;
  return len;
}

int WifiLink::read() {
  if (rxPos_ >= rxN_) {                           // next datagram, if one came
    rxN_ = rxPos_ = 0;
    if (wifiUdp.parsePacket() <= 0) return -1;
    wifiPeerIp   = wifiUdp.remoteIP();            // answer whoever talks to us
    wifiPeerPort = wifiUdp.remotePort();
    havePeer_ = true;
    const int n = wifiUdp.read(rx_, sizeof(rx_));
    if (n <= 0) return -1;
    rxN_ = (size_t)n;
  }
  return rx_[rxPos_++];
}

void WifiLink::lock() {
  wifiTxMutex.lock();
  txN_ = 0;
  txOver_ = false;
}

void WifiLink::unlock() {
  if (havePeer_ && txN_ && !txOver_) {
    wifiUdp.beginPacket(wifiPeerIp, wifiPeerPort);
    wifiUdp.write(tx_, txN_);
    wifiUdp.endPacket();
  }
  txN_ = 0;
  wifiTxMutex.unlock();
}

}  // namespace comm
