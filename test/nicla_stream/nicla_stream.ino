// Nicla Vision -> WiFi streamer: IMU + ToF ("lidar") + camera.
// As simple as possible, no safety checks.
//
// Speed vs calibration:
//   - normal stream = SMALLEST resolution (QQVGA 160x120) at HIGH frame rate.
//   - if the client sends 'F', the camera re-inits to FULL VGA 640x480 for
//     ONE frame (for calibration), then switches back to the fast mode.
//
// GC2145: 2 MP (native 1616x1232); smaller res = higher fps. RGB565 output.
// IMU LSM6DSOX on a dedicated internal SPI bus; ToF VL53L1CB on internal I2C (Wire1).
//
// Protocol (TCP): "IMU ax ay az gx gy gz TOF dist FRAME WxH len\n" + RGB565 bytes.

#include <WiFi.h>
#include <Wire.h>
#include "Arduino_LSM6DSOX.h"
#include "VL53L1X.h"
#include "camera.h"
#include "gc2145.h"

// ---- config ----
const char* SSID = "Penturas (2)";
const char* PASS = "penturas3";
const int   PORT = 8080;

const int BPP = 2;                         // RGB565

const int STREAM_W = 160, STREAM_H = 120;  // fast streaming size (smallest)
const int STREAM_FPS = 60;                 // request high fps (driver clamps)
const int FULL_W   = 640, FULL_H   = 480;  // calibration capture size

const uint32_t STREAM_BYTES = (uint32_t)STREAM_W * STREAM_H * BPP;
const uint32_t FULL_BYTES   = (uint32_t)FULL_W  * FULL_H  * BPP;

// ---- objects ----
WiFiServer server(PORT);
VL53L1X    tof;
GC2145     gc;
Camera     cam(gc);
FrameBuffer fb;

void setupWifi() {
  while (WiFi.begin(SSID, PASS) != WL_CONNECTED) {
    Serial.print("wifi status="); Serial.println(WiFi.status());
    delay(1000);
  }
  Serial.print("connected, IP "); Serial.println(WiFi.localIP());
  server.begin();
}

void setupSensors() {
  IMU.begin();

  Wire1.begin();
  tof.setBus(&Wire1);
  tof.init();
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);

  cam.begin(CAMERA_R160x120, CAMERA_RGB565, STREAM_FPS);  // start in fast mode
}

void sendHeader(WiFiClient& client, int w, int h, uint32_t len,
                float ax, float ay, float az, float gx, float gy, float gz,
                uint16_t dist) {
  client.print("IMU ");
  client.print(ax); client.print(' ');
  client.print(ay); client.print(' ');
  client.print(az); client.print(' ');
  client.print(gx); client.print(' ');
  client.print(gy); client.print(' ');
  client.print(gz); client.print(' ');
  client.print("TOF "); client.print(dist); client.print(' ');
  client.print("FRAME "); client.print(w); client.print('x');
  client.print(h); client.print(' ');
  client.print(len); client.print('\n');
}

void sendStream(WiFiClient& client) {
  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  uint16_t dist = tof.read();

  cam.grabFrame(fb, 3000);
  sendHeader(client, STREAM_W, STREAM_H, STREAM_BYTES, ax, ay, az, gx, gy, gz, dist);
  client.write(fb.getBuffer(), STREAM_BYTES);
}

// re-init to VGA, grab + send one full frame, then back to fast mode
void sendFull(WiFiClient& client) {
  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  uint16_t dist = tof.read();

  cam.begin(CAMERA_R640x480, CAMERA_RGB565, 30);
  cam.grabFrame(fb, 3000);
  sendHeader(client, FULL_W, FULL_H, FULL_BYTES, ax, ay, az, gx, gy, gz, dist);
  client.write(fb.getBuffer(), FULL_BYTES);
  cam.begin(CAMERA_R160x120, CAMERA_RGB565, STREAM_FPS);  // back to fast mode
  Serial.println("sent full frame");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("boot");
  setupWifi();
  setupSensors();
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;
  Serial.println("client connected");

  while (client.connected()) {
    bool full = false;
    while (client.available()) {
      if (client.read() == 'F') full = true;
    }
    if (full) sendFull(client);
    else      sendStream(client);
  }

  client.stop();
  Serial.println("client disconnected");
}
