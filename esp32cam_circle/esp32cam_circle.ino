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

/*
  Fixed address. Leave STATIC_IP empty for DHCP; set all three to pin
  the camera to a known address - the INDI driver and push_to.py
  defaults assume the HA reader at 192.168.4.1 and the DEC reader at
  192.168.4.2. All three must be set for static mode to engage.
*/
const char* STATIC_IP   = "192.168.1.41";
const char* STATIC_GW   = "192.168.1.1";
const char* STATIC_SN   = "255.255.255.0";

WebServer server(80);
camera_fb_t* fb = NULL;

bool camReady = false;
esp_err_t camErr = ESP_FAIL;
unsigned long camInitCount = 0;
unsigned long bmpGrabMs = 0;
unsigned long bmpSendMs = 0;
unsigned long bmpTotalMs = 0;
unsigned long bmpCount = 0;

/*
  Bench heartbeat: blinks the on-board (flash) LED without blocking.
  Set to 0 to disable (recommended for the dome: the LED is bright and
  points at the circle).
*/
#define HEARTBEAT_LED_MS 0
#define HEARTBEAT_LED_PIN 4

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
  int rStart = (int)(BAND_ROW_START * fb->height / 480.0f + 0.5f);
  int rEnd = (int)(BAND_ROW_END * fb->height / 480.0f + 0.5f);
  int rows = rEnd - rStart;
  if (rStart < 0 || rows <= 0 || rEnd > (int)fb->height ||
          fb->len < (size_t)(rEnd * w)) {
    esp_camera_fb_return(fb);
    return false;
  }

  static uint8_t prof[1600];
  static uint8_t hist[256];
  memset(hist, 0, sizeof(hist));
  int maxCols = (w < (int)sizeof(prof)) ? w : (int)sizeof(prof);

  for (int x = 0; x < maxCols; x++) {
    unsigned long acc = 0;
    const uint8_t* line = fb->buf + (size_t)rStart * w + x;
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
  /*
    Geometry is calibrated for VGA (CAL_CELL_PX at 640 px width);
    scale if the camera fell back to a smaller frame (PSRAM-less
    modules get QVGA so the grayscale framebuffer fits in DRAM).
  */
  int cellPx = (int)(CAL_CELL_PX * fb->width / 640.0f + 0.5f);
  if (cellPx < 1) cellPx = 1;
  if (markerStart < 0 || markerW < 11 * cellPx || markerW > 13 * cellPx) {
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
  if (!camReady) {
    server.send(503, "text/plain", "ERROR=camera not ready err=0x" +
                String((unsigned)camErr, HEX) + "\n");
    return;
  }
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
  if (!camReady) {
    server.send(503, "text/plain", "ERROR=camera not ready err=0x" +
                String((unsigned)camErr, HEX) + "\n");
    return;
  }
  if (!decodeCircle(&code)) {
    server.send(503, "text/plain", "ERROR=decode failed\n");
    return;
  }
  server.send(200, "text/plain", String(code) + "\n");
}

esp_err_t cameraInit() {
  /*
    Blob goes to PSRAM when the module has one (VGA grayscale needs a
    ~300 kB contiguous buffer, too big for DRAM); PSRAM-less clones get
    QVGA so the framebuffer fits in DRAM (decode scales geometry).
  */
  bool psram = psramFound();

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
  config.frame_size = psram ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (camReady) {
    esp_camera_deinit();
    camReady = false;
  }
  esp_err_t err = esp_camera_init(&config);
  camErr = err;
  camReady = (err == ESP_OK);
  camInitCount++;
  if (camReady) {
    sensor_t* s = esp_camera_sensor_get();
    /*
      Fixed exposure: AEC/AGC off, manual AEC value (OV3660/OV2640:
      0..1200). Tune with /debug's frame grab: dark cells should land
      well under ~100, white cells well over ~180 (see README).
    */
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, 0);
    s->set_awb_gain(s, 0);
    s->set_wb_mode(s, 0);
    s->set_special_effect(s, 0);
  }
  if (!camReady) {
    Serial.printf("camera init failed 0x%x (try %lu)\n", err, camInitCount);
  }
  return err;
}

void handleDebug() {
  String msg;
  msg += "AXIS=";
  msg += (CIRCLE_AXIS == AXIS_HA) ? "HA" : "DEC";
  msg += "\n";
  msg += "cam_ready=" + String(camReady ? "yes" : "no") + "\n";
  msg += "cam_init_err=0x" + String((unsigned)camErr, HEX) + "\n";
  msg += "cam_init_tries=" + String(camInitCount) + "\n";
  msg += "psram_size=" + String(ESP.getPsramSize()) + "\n";
  msg += "psram_free=" + String(ESP.getFreePsram()) + "\n";
  msg += "heap_free=" + String(ESP.getFreeHeap()) + "\n";
  msg += "ip=" + WiFi.localIP().toString() + "\n";
  msg += "rssi=" + String(WiFi.RSSI()) + "\n";
  if (bmpCount) {
    msg += "bmp_frames=" + String(bmpCount) + " grab_ms=" +
           String(bmpGrabMs) + " send_ms=" + String(bmpSendMs) +
           " total_ms=" + String(bmpTotalMs) + "\n";
  }
  if (camReady) {
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
      msg += "sensor=0x" + String(s->id.PID, HEX);
      msg += " ver 0x" + String(s->id.VER, HEX);
      msg += " mid 0x" + String(s->id.MIDL, HEX);
      msg += String(s->id.PID) == String(14036) ? " (OV3660)" : "";
      msg += String(s->id.PID) == String(9720) ? " (OV2640)" : "";
      msg += "\n";
    }
    camera_fb_t* d = esp_camera_fb_get();
    if (d) {
      msg += "frame=" + String(d->width) + "x" + String(d->height) +
             " len=" + String(d->len) + "\n";
      /* Level statistics over the calibrated band rows. */
      int bw = d->width;
      int rs = (int)(BAND_ROW_START * d->height / 480.0f + 0.5f);
      int re = (int)(BAND_ROW_END * d->height / 480.0f + 0.5f);
      if (rs < 0) rs = 0;
      if (re > (int)d->height) re = d->height;
      if (re > rs && d->len >= (size_t)(re * bw)) {
        int brow = re - rs;
        unsigned long mn = 255, mx = 0, sum = 0, cnt = 0;
        for (int y = rs; y < re; y++) {
          const uint8_t* line = d->buf + (size_t)y * bw;
          for (int x = 0; x < bw; x++) {
            unsigned v = line[x];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
            cnt++;
          }
        }
        msg += "band_rows=" + String(rs) + ".." + String(re) + "\n";
        msg += "band_min=" + String(mn) + " band_max=" + String(mx) +
               " band_mean=" + String(sum / cnt) + "\n";
      }
      esp_camera_fb_return(d);
    } else {
      msg += "frame=GRAB FAILED\n";
    }
  }
  server.send(200, "text/plain", msg);
}

void handleReinit() {
  esp_err_t err = cameraInit();
  if (err == ESP_OK) {
    server.send(200, "text/plain", "camera initialized\n");
  } else {
    server.send(503, "text/plain", "ERROR=camera init failed err=0x" +
                String((unsigned)err, HEX) + "\n");
  }
}

/*
  Raw TCP throughput test: streams n bytes of 0x00, no camera involved.
  'time curl -o /dev/null http://<ip>/fill?n=300000' separates the radio
  link from everything else. lwip's send buffer is only 5744 bytes in the
  prebuilt core, so each burst waits for the client's ACK; a healthy
  link still manages several MB/s, a bad one stalls ~200 ms per segment.
*/
void handleFill() {
  long n = server.hasArg("n") ? server.arg("n").toInt() : 100000;
  if (n < 0) n = 100000;
  if (n > 2000000) n = 2000000;
  static uint8_t zeros[1436];       /* one full TCP segment */
  memset(zeros, 0, sizeof(zeros));
  server.setContentLength((size_t)n);
  server.send(200, "application/octet-stream", "");
  server.client().setNoDelay(true);
  size_t left = (size_t)n;
  while (left > 0) {
    size_t want = left < sizeof(zeros) ? left : sizeof(zeros);
    size_t got = server.client().write(zeros, want);
    if (got == 0) break;            /* client gone */
    left -= got;
  }
}

/*
  Current frame as a small JPEG. The grayscale BMP is ~301 kB and, at the
  moment, painfully slow to push; the JPEG is 10-20x smaller for aiming
  and focusing. The sensor is switched to JPEG just for this request and
  back to grayscale afterwards so /pos keeps decoding properly.
*/
void handleJpg() {
  if (!camReady) {
    server.send(503, "text/plain", "ERROR=camera not ready err=0x" +
                String((unsigned)camErr, HEX) + "\n");
    return;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    server.send(503, "text/plain", "ERROR=no sensor\n");
    return;
  }
  camera_fb_t* f = esp_camera_fb_get();   /* drop stale grayscale frame */
  if (f) esp_camera_fb_return(f);
  s->set_pixformat(s, PIXFORMAT_JPEG);
  camera_fb_t* j = NULL;
  for (int tries = 0; tries < 4; tries++) {
    delay(120);                           /* let JPEG frames stream in */
    j = esp_camera_fb_get();
    if (j && j->len >= 2 && j->buf[0] == 0xFF && j->buf[1] == 0xD8) break;
    if (j) esp_camera_fb_return(j);
    j = NULL;
  }
  s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
  if (!j) {
    server.send(503, "text/plain", "ERROR=jpeg grab failed\n");
    return;
  }
  unsigned long t0 = millis();
  size_t jlen = j->len;
  server.setContentLength(j->len);
  server.send(200, "image/jpeg", "");
  server.client().setNoDelay(true);
  server.client().write(j->buf, j->len);
  esp_camera_fb_return(j);
  delay(120);                             /* drain one JPEG frame */
  f = esp_camera_fb_get();
  if (f) {
    esp_camera_fb_return(f);
    f = esp_camera_fb_get();              /* first grayscale frame */
    if (f) esp_camera_fb_return(f);
  }
  Serial.printf("jpg: %u bytes in %lu ms\n", (unsigned)jlen, millis() - t0);
}

/* Current frame as an 8-bit grayscale BMP, viewable in a browser. */
void handleBmp() {
  if (!camReady) {
    server.send(503, "text/plain", "ERROR=camera not ready err=0x" +
                String((unsigned)camErr, HEX) + "\n");
    return;
  }
  unsigned long t0 = millis();
  camera_fb_t* d = esp_camera_fb_get();
  if (!d) {
    server.send(503, "text/plain", "ERROR=grab failed\n");
    return;
  }
  unsigned long t1 = millis();
  int w = d->width;
  int h = d->height;
  uint32_t rowSize = (w + 3) & ~3u;
  uint32_t dataOff = 54u + 1024u;
  uint32_t fileSize = dataOff + rowSize * (uint32_t)h;

  uint8_t hdr[54];
  uint32_t u32 = fileSize;
  int32_t i32;
  hdr[0] = 'B';
  hdr[1] = 'M';
  memcpy(hdr + 2, &u32, 4);
  u32 = 0;
  memcpy(hdr + 6, &u32, 4);
  u32 = dataOff;
  memcpy(hdr + 10, &u32, 4);
  u32 = 40;
  memcpy(hdr + 14, &u32, 4);
  i32 = w;
  memcpy(hdr + 18, &i32, 4);
  i32 = -h;  /* negative height = top-down rows */
  memcpy(hdr + 22, &i32, 4);
  uint16_t u16 = 1;
  memcpy(hdr + 26, &u16, 2);
  u16 = 8;
  memcpy(hdr + 28, &u16, 2);
  u32 = 0;
  memcpy(hdr + 30, &u32, 4);
  u32 = rowSize * (uint32_t)h;
  memcpy(hdr + 34, &u32, 4);
  u32 = 0;
  memcpy(hdr + 38, &u32, 4);
  memcpy(hdr + 42, &u32, 4);
  u32 = 256;
  memcpy(hdr + 46, &u32, 4);
  memcpy(hdr + 50, &u32, 4);

  static uint8_t pal[1024];
  for (int i = 0; i < 256; i++) {
    pal[i * 4 + 0] = (uint8_t)i;
    pal[i * 4 + 1] = (uint8_t)i;
    pal[i * 4 + 2] = (uint8_t)i;
    pal[i * 4 + 3] = 0;
  }

  static uint8_t rowBuf[1600];
  /*
    One sendContent per 640-byte row + Nagle turns /bmp into a minute
    of 40ms-stalled writes. Batch rows into big chunks and disable
    Nagle on the socket.
  */
  static uint8_t chunk[8192];
  uint32_t chunkLen = 0;
  server.setContentLength(fileSize);
  server.send(200, "image/bmp", "");
  server.client().setNoDelay(true);
  server.sendContent((const char*)hdr, sizeof(hdr));
  server.sendContent((const char*)pal, sizeof(pal));
  for (int y = 0; y < h; y++) {
    memcpy(rowBuf, d->buf + (size_t)y * w, w);
    if (rowSize > (uint32_t)w) memset(rowBuf + w, 0, rowSize - w);
    memcpy(chunk + chunkLen, rowBuf, rowSize);
    chunkLen += rowSize;
    if (chunkLen > sizeof(chunk) - rowSize || y == h - 1) {
      server.sendContent((const char*)chunk, chunkLen);
      chunkLen = 0;
    }
    if (!server.client().connected()) {
      break;
    }
  }
  esp_camera_fb_return(d);
  bmpGrabMs = t1 - t0;
  bmpSendMs = millis() - t1;
  bmpTotalMs = millis() - t0;
  bmpCount++;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  /*
    Modem sleep (default on) throttles TCP badly; a circle reader is
    mains powered - keep the radio awake.
  */
  WiFi.setSleep(false);
  if (STATIC_IP[0] && STATIC_GW[0] && STATIC_SN[0])
  {
    IPAddress ip, gw, sn;
    if (!ip.fromString(STATIC_IP) || !gw.fromString(STATIC_GW) ||
            !sn.fromString(STATIC_SN))
    {
      Serial.println("bad STATIC_IP/STATIC_GW/STATIC_SN, falling back to DHCP");
    }
    else
    {
      WiFi.config(ip, gw, sn);
      Serial.printf("static config: %s gw %s\n", STATIC_IP, STATIC_GW);
    }
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  server.on("/pos", HTTP_GET, handlePos);
  server.on("/raw", HTTP_GET, handleRaw);
  server.on("/debug", HTTP_GET, handleDebug);
  server.on("/bmp", HTTP_GET, handleBmp);
  server.on("/jpg", HTTP_GET, handleJpg);
  server.on("/fill", HTTP_GET, handleFill);
  server.on("/init", HTTP_GET, handleReinit);
  server.begin();
  Serial.println("web server up, initializing camera");

  cameraInit();
}

void loop() {
  server.handleClient();

#if HEARTBEAT_LED_MS > 0
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  unsigned long now = millis();
  if (now - lastBlink >= HEARTBEAT_LED_MS) {
    lastBlink = now;
    ledOn = !ledOn;
    digitalWrite(HEARTBEAT_LED_PIN, ledOn);
  }
#endif
}
