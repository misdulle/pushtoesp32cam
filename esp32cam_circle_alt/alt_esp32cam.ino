// Step 1: camera bring-up only. No decoding yet.
// Browse to http://<ip>/  to see a live grayscale frame (reload to refresh),
// use it to focus the lens (unscrew it for close range) and frame a strip.
// Luma min/max/mean of each served frame is printed on the serial console.
//
// UNTESTED on hardware -- written against the AI-Thinker pin map and the
// esp32-camera driver API. If the build complains about pin_sccb_sda /
// pin_sccb_scl, your core is older: rename them to pin_sscb_sda / pin_sscb_scl.

#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

static const char *WIFI_SSID = "ed";
static const char *WIFI_PASS = "gsin2hhlx48f6v25jlgz";

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

WebServer server(80);

static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_VGA;          // 640x480
  c.jpeg_quality = 12;                   // unused in grayscale mode
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("esp_camera_init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

static void handleRoot() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "capture failed");
    return;
  }

  uint8_t lo = 255, hi = 0;
  uint64_t sum = 0;
  for (size_t i = 0; i < fb->len; i++) {
    uint8_t v = fb->buf[i];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    sum += v;
  }
  Serial.printf("frame %ux%u len=%u luma min=%u max=%u mean=%.1f\n",
                fb->width, fb->height, (unsigned)fb->len, lo, hi,
                fb->len ? (double)sum / fb->len : 0.0);

  uint8_t *jpg = nullptr;
  size_t jlen = 0;
  bool ok = frame2jpg(fb, 85, &jpg, &jlen);
  esp_camera_fb_return(fb);
  if (!ok) {
    server.send(500, "text/plain", "jpeg conversion failed");
    return;
  }

  server.setContentLength(jlen);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char *)jpg, jlen);
  free(jpg);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initCamera()) {
    Serial.println("Camera init failed -- check wiring/power (needs a solid 5V supply).");
    return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nOpen http://%s/\n", WiFi.localIP().toString().c_str());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}
