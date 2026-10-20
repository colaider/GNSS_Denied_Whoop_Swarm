// Nicla Vision -> WiFi: CAMERA-ONLY calibration streamer.
// Streams a small preview; when the client sends 'F' it returns ONE full
// raw frame, which is what you save for calibration.
//
// Why these settings (from the Arduino Camera + GC2145 library source):
//  * Format = CAMERA_BAYER (raw, 1 byte/px). CAMERA_GRAYSCALE is broken on the
//    GC2145: the sensor sends Bayer but the DCMI drops every other byte.
//    Raw also skips the ISP sharpening/denoise, which is better for calibration.
//  * Resolution = VGA 640x480 -> 307 KB, fits the 512 KB SRAM bank
//    (RGB565 VGA = 614 KB does not, that was the red-LED crash).
//  * Every resolution is a DIFFERENT centred crop of the 1600x1200 sensor:
//        160x120 : 640x480 window,  subsample 4
//        320x240 : 960x720 window,  subsample 3
//        640x480 : 1280x960 window, subsample 2
//    Calibrate the mode you will fly, or convert intrinsics with these ratios.
//  * Capture is snapshot mode: the frame grabbed after 'F' is a fresh exposure.
//
// Protocol (TCP): "FRAME <kind> <w> <h> <len>\n" + <len> bytes, 1 byte/px
//   kind P = preview 160x120 (one Bayer phase -> clean grey)
//   kind F = full 640x480 raw Bayer

#include <WiFi.h>
#include "camera.h"
#include "gc2145.h"

// ---- config ----
const char* SSID = "Penturas (2)";
const char* PASS = "penturas3";
const int   PORT = 8080;

const int RES    = CAMERA_R640x480;          // calibration mode
const int FULL_W = 640, FULL_H = 480;        // must match RES
const int STEP   = 4;                        // even step keeps one Bayer phase
const int PREV_W = FULL_W / STEP;            // 160
const int PREV_H = FULL_H / STEP;            // 120

const uint32_t FULL_BYTES = (uint32_t)FULL_W * FULL_H;   // Bayer = 1 byte/px
const uint32_t PREV_BYTES = (uint32_t)PREV_W * PREV_H;

// ---- objects ----
WiFiServer  server(PORT);
GC2145      gc;
Camera      cam(gc);
FrameBuffer fb;

uint8_t prev[PREV_BYTES];

void makePreview(const uint8_t* src) {
  int di = 0;
  for (int y = 0; y < PREV_H; y++)
    for (int x = 0; x < PREV_W; x++)
      prev[di++] = src[(y * STEP) * FULL_W + x * STEP];
}

void sendAll(WiFiClient& client, const uint8_t* p, uint32_t n) {
  while (n) {                                // chunked so big frames arrive whole
    int k = client.write(p, n > 4096 ? 4096 : n);
    if (k <= 0) return;
    p += k; n -= k;
  }
}

void sendFrame(WiFiClient& client, bool full) {
  if (cam.grabFrame(fb, 3000) != 0) {
    Serial.println("grab failed");
    return;
  }
  const uint8_t* img = fb.getBuffer();

  char kind = full ? 'F' : 'P';
  int w = full ? FULL_W : PREV_W;
  int h = full ? FULL_H : PREV_H;
  uint32_t len = full ? FULL_BYTES : PREV_BYTES;
  if (!full) makePreview(img);

  client.print("FRAME "); client.print(kind); client.print(' ');
  client.print(w); client.print(' ');
  client.print(h); client.print(' ');
  client.print(len); client.print('\n');
  sendAll(client, full ? img : prev, len);

  if (full) Serial.println("sent full frame");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);   // don't hang without USB
  Serial.println("boot");

  // own the frame buffer: a RAM shortage is reported instead of hard-faulting
  uint8_t* mem = (uint8_t*)malloc(FULL_BYTES + 32);
  if (!mem) { Serial.println("not enough RAM for full frame"); while (1); }
  fb.setBuffer((uint8_t*)(((uint32_t)mem + 31) & ~31UL));   // 32-byte aligned for DMA

  while (WiFi.begin(SSID, PASS) != WL_CONNECTED) {
    Serial.print("wifi status="); Serial.println(WiFi.status());
    delay(1000);
  }
  Serial.print("connected, IP "); Serial.println(WiFi.localIP());
  server.begin();

  if (!cam.begin(RES, CAMERA_BAYER, 30)) Serial.println("camera init failed");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;
  Serial.println("client connected");

  while (client.connected()) {
    bool full = false;
    while (client.available())
      if (client.read() == 'F') full = true;
    sendFrame(client, full);
  }

  client.stop();
  Serial.println("client disconnected");
}
