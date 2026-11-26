#pragma once

// The Nicla's unique ID: the STM32H747's 96-bit UID, written at the factory (RM0399, "Unique device ID",
// address 0x1FF1E800). Hex like Betaflight's mcu_id: word 0, 1, 2, 8 digits each, lower case -> 24 characters.
// drones/drones.h picks this drone (calibration, gains, ...) by it.
//
// Only the M7 can read that address: the M4 hard-faults on it (it is in the M7's part of the memory map; ST
// forum, "UID_BASE access from STM32H745ZI CM4 core"; 25.11 it killed the M4 at boot). A copy in shared SRAM4
// did not reach the M4 either (25.11: it read 0). So the M7 sends it as a packet, msg::PKT_NICLA_ID, at boot
// (m7Runner, 5 x 1 s apart), and the M4 keeps it with setNiclaUid(). Until it arrives the M4 ID is 0: unknown board.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace shared {

#if defined(CORE_CM4)
inline uint32_t* niclaUidStore() { static uint32_t w[3] = {0, 0, 0}; return w; }

// The M4, from the M7's packet. true if it changed.
inline bool setNiclaUid(const uint32_t w[3]) {
  uint32_t* s = niclaUidStore();
  if (memcmp(s, w, 12) == 0) return false;
  memcpy(s, w, 12);
  return true;
}
#endif

inline void niclaUid(uint32_t w[3]) {
#if defined(CORE_CM4)
  memcpy(w, niclaUidStore(), 12);
#else
  const volatile uint32_t* p = (const volatile uint32_t*)0x1FF1E800UL;
  for (int i = 0; i < 3; i++) w[i] = p[i];
#endif
}

inline const char* niclaUidHex() {             // "004800333432510132353538"
  static char s[25];
  uint32_t w[3];
  niclaUid(w);
  snprintf(s, sizeof(s), "%08lx%08lx%08lx", (unsigned long)w[0], (unsigned long)w[1], (unsigned long)w[2]);
  return s;
}

}
