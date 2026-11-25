/*
  esp32cam_circle.ino - graduated-circle reader for the push-to system.

  An AI-Thinker ESP32-CAM watches a code band fixed around one of the
  telescope circles. The band is the GC9 scheme:
    - 9 fixed-pitch data cells carrying a 9-bit Gray code
      (288 codes on the HA circle: one cell = 5 min of hour angle,
       360 codes on the DEC circle: one cell = 30 arcmin)
    - one dark start marker 12 cells wide, followed by ONE white gap
      cell, so the marker can never merge with dark data cells. The
      longest possible dark run of data cells is 9 (all-ones), so the
      12-cell marker is always the widest dark run in the view.
      Adjacent code blocks are separated by at least one white cell.

  Marker: 12 cells wide. Data cells: pitch CAL_CELL_PX px.
  Guard: widest dark run must be ~12 cells, else decode fails.

  Decode per frame: threshold the code band, find the widest dark run
  (the marker), sample 9 fixed-width windows after it, threshold each
  average to a bit, Gray-decode, map to HA hours or DEC degrees.

  Flash with CIRCLE_AXIS set to AXIS_HA or AXIS_DEC, then point the
  host console at http://<this-ip>/pos which answers:

      AXIS=HA|DEC
      CODE=<decoded Gray index, 0..287 or 0..359>
      VALUE=<HA hours | DEC degrees>

  Band geometry (row range, expected cell width in pixels) depends on
  camera-to-circle distance: calibrate CAL_CELL_PX and BAND_* first.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

#define AXIS_HA  0
#define AXIS_DEC 1
/*
  Compile-time axis selection: edit this line, or leave it and pass
  -DCIRCLE_AXIS=1 (AXIS_DEC) with
  arduino-cli compile --build-property "compiler.cpp.extra_flags=..."
  so the same file serves both cameras.
*/
#ifndef CIRCLE_AXIS
#define CIRCLE_AXIS AXIS_HA
#endif

#define N_BITS 9

/*
  AI-Thinker ESP32-CAM pin map. The core does not define these; every
  sketch carries its own (see the CameraWebServer example's
  camera_pins.h). Guarded so other maps can replace them via -D.
*/
#ifndef PWDN_GPIO_NUM
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
#endif

#define MARKER_CELLS 12
#define GAP_CELLS 1
#define CAL_CELL_PX 12
#define BAND_ROW_START 220
#define BAND_ROW_END 260

const char* WIFI_SSID = "your-ssid";
const char* WIFI_PASS = "your-password";

WebServer server(80);
camera_fb_t* fb = NULL;

float angleFromCode(int n) {
  if (CIRCLE_AXIS == AXIS_HA) {
    if (n < 0 || n >= 288) return NAN;
    return (n * 5.0f) / 60.0f;
  }
  if (n < 0 || n >= 360) return NAN;
  return -90.0f + n * 0.5f;
}

uint16_t grayToBinary(uint16_t g) {
  uint16_t b = g;
  b ^= (b >> 8);
  b ^= (b >> 4);
  b ^= (b >> 2);
  b ^= (b >> 1);
  return b & ((1 << N_BITS) - 1);
}

int otsuThreshold(const uint8_t* hist, int total) {
  float sum = 0.0f;
  for (int i = 0; i < 256; i++) sum += (float)i * hist[i];
  float sumB = 0.0f;
  int wB = 0;
  int wF = total;
  float maxVar = -1.0f;
  int best = 127;
  for (int t = 0; t < 256; t++) {
    wB += hist[t];
    if (wB == 0) continue;
    wF = total - wB;
    if (wF == 0) break;
    sumB += (float)t * hist[t];
    float mB = sumB / wB;
    float mF = (sum - sumB) / wF;
    float var = (float)wB * (float)wF * (mB - mF) * (mB - mF);
    if (var > maxVar) {
      maxVar = var;
      best = t;
    }
  }
  return best;
}

int findWidestDarkRun(const uint8_t* v, int len, uint8_t thr, int* width_out) {
  int bestStart = -1;
  int bestW = 0;
  int curStart = -1;
  for (int i = 0; i <= len; i++) {
    bool dark = (i < len) && (v[i] <= thr);
    if (dark && curStart < 0) {
      curStart = i;
    } else if (!dark && curStart >= 0) {
      int w = i - curStart;
      if (w > bestW) {
        bestW = w;
        bestStart = curStart;
      }
      curStart = -1;
    }
  }
  *width_out = bestW;
  return bestStart;
}

bool decodeCircle(int* code_out) {
  fb = esp_camera_fb_get();
  if (!fb) return false;

  int w = fb->width;
  int rows = BAND_ROW_END - BAND_ROW_START;
  if (fb->len < (size_t)(BAND_ROW_END * w)) {
    esp_camera_fb_return(fb);
    return false;
  }

  static uint8_t prof[1600];
  static uint8_t hist[256];
  memset(hist, 0, sizeof(hist));
  int maxCols = (w < (int)sizeof(prof)) ? w : (int)sizeof(prof);

  for (int x = 0; x < maxCols; x++) {
    unsigned long acc = 0;
    const uint8_t* line = fb->buf + (size_t)BAND_ROW_START * w + x;
    for (int y = 0; y < rows; y++) acc += line[(size_t)y * w];
    uint8_t v = (uint8_t)(acc / rows);
    prof[x] = v;
    hist[v]++;
  }

  int thr = otsuThreshold(hist, maxCols);

  float darkSum = 0.0f;
  long darkCnt = 0;
  float whiteSum = 0.0f;
  long whiteCnt = 0;
  for (int v = 0; v < 256; v++) {
    if (!hist[v]) continue;
    if (v <= thr) {
      darkSum += (float)v * hist[v];
      darkCnt += hist[v];
    } else {
      whiteSum += (float)v * hist[v];
      whiteCnt += hist[v];
    }
  }
  float bitThr = thr;
  if (darkCnt > 0 && whiteCnt > 0) {
    bitThr = 0.5f * (darkSum / darkCnt + whiteSum / whiteCnt);
  }

  int markerW = 0;
  int markerStart = findWidestDarkRun(prof, maxCols, thr, &markerW);
  if (markerStart < 0 || markerW < 11 * CAL_CELL_PX || markerW > 13 * CAL_CELL_PX) {
    esp_camera_fb_return(fb);
    return false;
  }

  float pitch = (float)markerW / MARKER_CELLS;
  uint16_t gray = 0;
  int bandStart = markerStart + markerW + (int)(GAP_CELLS * pitch + 0.5f);
  for (int i = 0; i < N_BITS; i++) {
    int x0 = bandStart + (int)(i * pitch + 0.5f);
    int x1 = bandStart + (int)((i + 1) * pitch + 0.5f);
    if (x0 < 0 || x1 <= x0 || x1 > maxCols) {
      esp_camera_fb_return(fb);
      return false;
    }
    unsigned long acc = 0;
    for (int dx = x0; dx < x1; dx++) acc += prof[dx];
    float avg = (float)acc / (x1 - x0);
    if (avg <= bitThr) gray |= (1 << (N_BITS - 1 - i));
  }

  esp_camera_fb_return(fb);
  *code_out = (int)grayToBinary(gray);
  return true;
}

void handlePos() {
  int code = -1;
  if (!decodeCircle(&code)) {
    server.send(503, "text/plain", "ERROR=decode failed\n");
    return;
  }
  float value = angleFromCode(code);
  if (isnan(value)) {
    String msg = "ERROR=code out of range " + String(code) + "\n";
    server.send(503, "text/plain", msg);
    return;
  }
  String msg = "AXIS=";
  msg += (CIRCLE_AXIS == AXIS_HA) ? "HA" : "DEC";
  msg += "\nCODE=" + String(code);
  msg += "\nVALUE=";
  msg += String(value, 5);
  msg += "\n";
  server.send(200, "text/plain", msg);
}

void handleRaw() {
  int code = -1;
  if (!decodeCircle(&code)) {
    server.send(503, "text/plain", "ERROR=decode failed\n");
    return;
  }
  server.send(200, "text/plain", String(code) + "\n");
}

void setup() {
  Serial.begin(115200);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init failed 0x%x\n", err);
    delay(5000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  /*
    Fixed exposure: AEC/AGC off, manual AEC value (OV2640: 0..1200).
    Tune with /raw plus a photo of the band: dark cells should land
    well under ~100, white cells well over ~180 (see README).
  */
  s->set_exposure_ctrl(s, 0);
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 0);
  s->set_awb_gain(s, 0);
  s->set_wb_mode(s, 0);
  s->set_special_effect(s, 0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  server.on("/pos", HTTP_GET, handlePos);
  server.on("/raw", HTTP_GET, handleRaw);
  server.begin();
}

void loop() {
  server.handleClient();
}
