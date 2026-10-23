#include "headers/usbLink.h"

#include <Arduino.h>

#include "mbed.h"

namespace comm {

static rtos::Mutex usbTxMutex;                          // one USB port: one mutex

bool UsbLink::begin() {
  Serial.begin(921600);                                 // USB: the baud rate is ignored
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);    // give the PC a moment to open the port
  return true;
}

size_t UsbLink::write(const uint8_t* data, size_t len) {
  return Serial.write(data, len);
}

int UsbLink::read() {
  return Serial.available() ? Serial.read() : -1;
}

void UsbLink::lock() {
  usbTxMutex.lock();
}

void UsbLink::unlock() {
  usbTxMutex.unlock();
}

}  // namespace comm
