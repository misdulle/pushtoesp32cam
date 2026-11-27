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

  Band geometry (strip-column range, expected cell pitch in pixels)
  depends on camera-to-circle distance: calibrate CAL_CELL_PX and
  STRIP_COL_* first.
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

/*
  Working framesize when PSRAM is available (decode geometry scales with
  the frame). SVGA (800x600) sustains ~5.6 fps with the OV3660 at 20 MHz
  XCLK / fixed exposure -- the biggest image that still meets the 5 fps
  target; XGA drops to ~2.8 fps. See GET /bench for the measured ladder.
*/
#ifndef CIRCLE_FRAMESIZE
#define CIRCLE_FRAMESIZE FRAMESIZE_SVGA
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
/*
  Geometry for radial code strips (parallel to the graduated bars),
  as on the real instrument: the mobile part moves VERTICALLY through
  the FOV (the tangential direction is the camera's Y axis), the
  radial direction is X. A code strip is 27 cells laid out along X
  ([2 lead][12 marker][1 gap][9 Gray, MSB at the strip's left/inner
  end][3 tail]), one cell tall in Y. The strips live in a radial band
  on the LEFT of the FOV; the graduated marks (1-min short, 5-min
  long, anchored at the rim) sit in a separate band to their RIGHT,
  each long mark at its strip's height. The fixed pointer line runs
  parallel to the marks (horizontal) at a fixed height near mid-FOV,
  from the rim (the marks' anchored right ends) to the FOV's right
  edge -- it never crosses the code band; the decoder reads the strip
  closest to that height.

  CAL_CELL_PX     radial cell pitch in px at 640 columns (VGA-
                  calibrated; scaled by width/640). Max ~21 at VGA,
                  ~27 at SVGA (27 cells must fit the strip region).
  STRIP_COL_*     columns covered by the strips' radial extent
                  (scaled the same way). The coverage scan runs over
                  these columns only, so the graduated marks (in
                  their own band) and the pointer (which only exists
                  right of the marks' rim) never enter it; the
                  run-height filter additionally rejects any thin
                  horizontal artifact.
  CIRCLE_STRIP_FLIP  set to 1 if the data cells read opposite to the
                  painted order (window 0 = MSB = the data cell
                  nearest the marker by default).
*/
#define CAL_CELL_PX 15
#define STRIP_COL_START 40
#define STRIP_COL_END 460
#ifndef CIRCLE_STRIP_FLIP
#define CIRCLE_STRIP_FLIP 0
#endif

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
size_t jpgLen = 0;
unsigned long jpgMs = 0;

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

int otsuThreshold(const uint32_t* hist, int total) {
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

/*
  Decode a radial code strip. The mobile pattern moves vertically in
  the FOV: the camera's Y axis is tangential, X radial. Steps: Otsu
  over the strip columns -> per-row dark coverage -> strip = the
  height-filtered coverage run closest to mid-FOV height (that is
  where the fixed pointer sits; the pointer itself stays right of the
  strip columns, and the height filter rejects any thin horizontal
  artifact) -> radial profile along the strip's rows -> Otsu +
  dark/white midpoint -> widest dark
  run (12-cell marker) with the 11..13-cell guard -> 9 Gray windows ->
  binary code.
*/
bool decodeFb(camera_fb_t* fb, int* code_out) {
  int w = fb->width;
  int h = fb->height;
  int cStart = (int)(STRIP_COL_START * w / 640.0f + 0.5f);
  int cEnd = (int)(STRIP_COL_END * w / 640.0f + 0.5f);
  int cols = cEnd - cStart;
  if (cStart < 0 || cols <= 0 || cEnd > w ||
          fb->len < (size_t)((size_t)h * w)) {
    esp_camera_fb_return(fb);
    return false;
  }
  /*
    Geometry is calibrated for VGA (CAL_CELL_PX at 640 columns); scale
    with the frame width (the radial direction is X). PSRAM-less
    modules get QVGA so the grayscale framebuffer fits in DRAM.
  */
  int cellPx = (int)(CAL_CELL_PX * w / 640.0f + 0.5f);
  if (cellPx < 1) cellPx = 1;

  /* uint32_t bins: a strip region can hold >65535 pixels of one level. */
  static uint32_t hist[256];
  static uint16_t rowcov[1200];
  static uint8_t prof[1600];
  memset(hist, 0, sizeof(hist));
  memset(rowcov, 0, sizeof(uint16_t) * h);

  /* Pass 1: level histogram over the strip columns -> global threshold. */
  for (int y = 0; y < h; y++) {
    const uint8_t* line = fb->buf + (size_t)y * w;
    for (int x = cStart; x < cEnd; x++) hist[line[x]]++;
  }
  int thr = otsuThreshold(hist, cols * h);

  /* Pass 2: dark coverage per row within the strip columns. */
  for (int y = 0; y < h; y++) {
    const uint8_t* line = fb->buf + (size_t)y * w;
    int cov = 0;
    for (int x = cStart; x < cEnd; x++) {
      if (line[x] <= thr) cov++;
    }
    rowcov[y] = (uint16_t)cov;
  }

  /*
    Strip rows: coverage runs of at least 30% of the columns (the
    marker alone gives 12/27 = 0.44 of the strip; the margin absorbs
    region-alignment slop) and at least 0.7 cells tall (the horizontal
    pointer line is thinner). Of the qualifying runs take the one
    closest to mid-height -- the fixed pointer selects the current
    strip.
  */
  int minCov = (int)(0.30f * cols);
  int minHeight = (int)(0.7f * cellPx);
  if (minHeight < 1) minHeight = 1;
  int stripY0 = -1, stripY1 = -1;
  long bestDist = -1;
  int rs = -1;
  for (int y = 0; y <= h; y++) {
    bool good = (y < h) && (rowcov[y] >= minCov);
    if (good && rs < 0) {
      rs = y;
    } else if (!good && rs >= 0) {
      if (y - rs >= minHeight) {
        long dist = (long)(rs + y) / 2 - h / 2;
        if (dist < 0) dist = -dist;
        if (bestDist < 0 || dist < bestDist) {
          bestDist = dist;
          stripY0 = rs;
          stripY1 = y;
        }
      }
      rs = -1;
    }
  }
  if (stripY0 < 0) {
    esp_camera_fb_return(fb);
    return false;
  }

  /* Radial profile along the strip's rows. */
  int profLen = cEnd - cStart;
  int sr = stripY1 - stripY0;
  for (int x = 0; x < profLen; x++) {
    unsigned long acc = 0;
    for (int y = 0; y < sr; y++) {
      acc += fb->buf[(size_t)(stripY0 + y) * w + cStart + x];
    }
    prof[x] = (uint8_t)(acc / sr);
  }

  memset(hist, 0, sizeof(hist));
  for (int y = 0; y < profLen; y++) hist[prof[y]]++;
  thr = otsuThreshold(hist, profLen);

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
  int markerStart = findWidestDarkRun(prof, profLen, thr, &markerW);
  if (markerStart < 0 || markerW < 11 * cellPx || markerW > 13 * cellPx) {
    esp_camera_fb_return(fb);
    return false;
  }

  float pitch = (float)markerW / MARKER_CELLS;
  uint16_t gray = 0;
  int bandStart = markerStart + markerW + (int)(GAP_CELLS * pitch + 0.5f);
  for (int i = 0; i < N_BITS; i++) {
    int y0 = bandStart + (int)(i * pitch + 0.5f);
    int y1 = bandStart + (int)((i + 1) * pitch + 0.5f);
    if (y0 < 0 || y1 <= y0 || y1 > profLen) {
      esp_camera_fb_return(fb);
      return false;
    }
    unsigned long acc = 0;
    for (int dy = y0; dy < y1; dy++) acc += prof[dy];
    float avg = (float)acc / (y1 - y0);
    if (avg <= bitThr) {
#if CIRCLE_STRIP_FLIP
      gray |= (1 << i);
#else
      gray |= (1 << (N_BITS - 1 - i));
#endif
    }
  }

  esp_camera_fb_return(fb);
  *code_out = (int)grayToBinary(gray);
  return true;
}

bool decodeCircle(int* code_out) {
  if (!camReady) return false;
  fb = esp_camera_fb_get();
  if (!fb) return false;
  /* decodeFb releases the buffer itself on every path */
  return decodeFb(fb, code_out);
}

/*
  Text response helper: mirrors every response on Serial, so the same
  information is available on the console (watch with dtr=off,rts=off --
  see README) and over HTTP.
*/
void reply(int code, String msg) {
  if (!msg.endsWith("\n")) msg += "\n";
  Serial.printf("[HTTP %d %s] %s", code, server.uri().c_str(), msg.c_str());
  server.send(code, "text/plain", msg);
}

void handlePos() {
  int code = -1;
  if (!camReady) {
    reply(503, "ERROR=camera not ready err=0x" +
          String((unsigned)camErr, HEX) + "\n");
    return;
  }
  if (!decodeCircle(&code)) {
    reply(503, "ERROR=decode failed\n");
    return;
  }
  float value = angleFromCode(code);
  if (isnan(value)) {
    String msg = "ERROR=code out of range " + String(code) + "\n";
    reply(503, msg);
    return;
  }
  String msg = "AXIS=";
  msg += (CIRCLE_AXIS == AXIS_HA) ? "HA" : "DEC";
  msg += "\nCODE=" + String(code);
  msg += "\nVALUE=";
  msg += String(value, 5);
  msg += "\n";
  reply(200, msg);
}

void handleRaw() {
  int code = -1;
  if (!camReady) {
    reply(503, "ERROR=camera not ready err=0x" +
          String((unsigned)camErr, HEX) + "\n");
    return;
  }
  if (!decodeCircle(&code)) {
    reply(503, "ERROR=decode failed\n");
    return;
  }
  reply(200, String(code) + "\n");
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
  config.frame_size = psram ? CIRCLE_FRAMESIZE : FRAMESIZE_QVGA;
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
  if (jpgLen) {
    msg += "jpg_len=" + String(jpgLen) + " jpg_ms=" + String(jpgMs) + "\n";
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
      /* Level statistics over the calibrated strip columns. */
      int bw = d->width;
      int bh = d->height;
      int cs = (int)(STRIP_COL_START * bw / 640.0f + 0.5f);
      int ce = (int)(STRIP_COL_END * bw / 640.0f + 0.5f);
      if (cs < 0) cs = 0;
      if (ce > bw) ce = bw;
      if (ce > cs && d->len >= (size_t)((size_t)bh * bw)) {
        unsigned long mn = 255, mx = 0, sum = 0, cnt = 0;
        for (int y = 0; y < bh; y++) {
          const uint8_t* line = d->buf + (size_t)y * bw;
          for (int x = cs; x < ce; x++) {
            unsigned v = line[x];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
            cnt++;
          }
        }
        msg += "band_cols=" + String(cs) + ".." + String(ce) + "\n";
        msg += "band_min=" + String(mn) + " band_max=" + String(mx) +
               " band_mean=" + String(sum / cnt) + "\n";
      }
      esp_camera_fb_return(d);
    } else {
      msg += "frame=GRAB FAILED\n";
    }
  }
  reply(200, msg);
}

void handleReinit() {
  esp_err_t err = cameraInit();
  if (err == ESP_OK) {
    reply(200, "camera initialized\n");
  } else {
    reply(503, "ERROR=camera init failed err=0x" +
          String((unsigned)err, HEX) + "\n");
  }
}

/* Framesize selector by name for /bench. */
static bool framesizeFromName(String n, framesize_t* out) {
  n.toLowerCase();
  if (n == "qqvga") *out = FRAMESIZE_QQVGA;
  else if (n == "qvga") *out = FRAMESIZE_QVGA;
  else if (n == "vga") *out = FRAMESIZE_VGA;
  else if (n == "svga") *out = FRAMESIZE_SVGA;
  else if (n == "xga") *out = FRAMESIZE_XGA;
  else if (n == "hd") *out = FRAMESIZE_HD;
  else if (n == "sxga") *out = FRAMESIZE_SXGA;
  else if (n == "uxga") *out = FRAMESIZE_UXGA;
  else return false;
  return true;
}

/*
  Timing benchmark: sensor read (fresh-frame wait) vs decode, at the
  current framesize or across the whole ladder with ?all=1.
    /bench                 measure current size
    /bench?fs=vga          measure after switching to VGA (restored after)
    /bench?fs=uxga&keep=1  switch and stay there
    /bench?all=1           sweep qqvga..uxga, restore original size
  The 'get_ms' figure is the wait for one FRESH frame after releasing the
  previous buffer (fb_count=1, GRAB_LATEST), i.e. the sensor period; the
  achievable continuous rate is 1000/get_ms fps.
*/
void handleBench() {
  if (!camReady) {
    reply(503, "ERROR=camera not ready err=0x" +
          String((unsigned)camErr, HEX) + "\n");
    return;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    reply(503, "ERROR=no sensor\n");
    return;
  }
  framesize_t origFs = (framesize_t)s->status.framesize;
  bool restore = true;

  String body = "";
  if (server.hasArg("all")) {
    static const framesize_t ladder[] = {
      FRAMESIZE_QQVGA, FRAMESIZE_QVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA,
      FRAMESIZE_XGA, FRAMESIZE_HD, FRAMESIZE_SXGA, FRAMESIZE_UXGA
    };
    for (unsigned i = 0; i < sizeof(ladder) / sizeof(ladder[0]); i++) {
      if (s->set_framesize(s, ladder[i]) != 0) {
        body += "fs=" + String((int)ladder[i]) + " set failed\n";
        continue;
      }
      delay(150);
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) esp_camera_fb_return(fb);
      unsigned long t0 = micros();
      fb = esp_camera_fb_get();
      unsigned long getUs = micros() - t0;
      if (!fb) {
        body += "get failed\n";
        continue;
      }
      unsigned long t1 = micros();
      int code = -1;
      bool ok = decodeFb(fb, &code);
      unsigned long decUs = micros() - t1;
      float getMs = getUs / 1000.0f;
      body += "fs=" + String((int)ladder[i]) +
              " " + String(fb->width) + "x" + String(fb->height) +
              " get_ms=" + String(getMs, 1) +
              " decode_ms=" + String(decUs / 1000.0f, 2) +
              " fps_max=" + String(1000.0f / getMs, 1) +
              " ok=" + (ok ? String("yes code=") + String(code) : String("no")) +
              "\n";
    }
    s->set_framesize(s, origFs);
    delay(150);
    reply(200, body);
    return;
  }

  if (server.hasArg("fs")) {
    framesize_t fs;
    if (!framesizeFromName(server.arg("fs"), &fs)) {
      reply(400, "ERROR=bad fs name\n");
      return;
    }
    if (s->set_framesize(s, fs) != 0) {
      reply(503, "ERROR=set_framesize failed\n");
      return;
    }
    if (server.hasArg("keep")) restore = false;
    delay(150);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  unsigned long t0 = micros();
  fb = esp_camera_fb_get();
  unsigned long getUs = micros() - t0;
  if (!fb) {
    if (restore) {
      s->set_framesize(s, origFs);
      delay(150);
    }
    reply(503, "ERROR=grab failed\n");
    return;
  }
  int fw = fb->width, fh = fb->height;
  unsigned long t1 = micros();
  int code = -1;
  bool ok = decodeFb(fb, &code);
  unsigned long decUs = micros() - t1;
  float getMs = getUs / 1000.0f;
  String msg = "frame=" + String(fw) + "x" + String(fh) +
               " get_ms=" + String(getMs, 1) +
               " decode_ms=" + String(decUs / 1000.0f, 2) +
               " fps_max=" + String(1000.0f / getMs, 1) + "\n";
  msg += ok ? ("decode=ok code=" + String(code) +
               " value=" + String(angleFromCode(code), 5) + "\n")
            : "decode=failed\n";
  if (restore) {
    s->set_framesize(s, origFs);
    delay(150);
  }
  reply(200, msg);
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
  Serial.printf("[/fill] %lu of %ld bytes sent\n",
                (unsigned long)((unsigned long)n - left), n);
}

/*
  Current frame as a small JPEG. The grayscale BMP is ~301 kB and slow to
  push; the frame2jpg() conversion (esp32-camera built-in) shrinks it 10-20x
  for aiming and focusing, with no pixformat switching -- /pos keeps decoding
  the same grayscale frames as always.
*/
void handleJpg() {
  if (!camReady) {
    reply(503, "ERROR=camera not ready err=0x" +
          String((unsigned)camErr, HEX) + "\n");
    return;
  }
  camera_fb_t* d = esp_camera_fb_get();
  if (!d) {
    reply(503, "ERROR=grab failed\n");
    return;
  }
  unsigned long t0 = millis();
  uint8_t* jpg = NULL;
  size_t jlen = 0;
  bool ok = frame2jpg(d, 85, &jpg, &jlen);
  esp_camera_fb_return(d);
  if (!ok || !jpg) {
    reply(503, "ERROR=jpeg conversion failed\n");
    return;
  }
  server.setContentLength(jlen);
  server.send(200, "image/jpeg", "");
  server.client().setNoDelay(true);
  server.client().write(jpg, jlen);
  free(jpg);
  jpgLen = jlen;
  jpgMs = millis() - t0;
  Serial.printf("[/jpg] %u bytes in %lu ms\n", (unsigned)jpgLen, jpgMs);
}

/* Current frame as an 8-bit grayscale BMP, viewable in a browser. */
void handleBmp() {
  if (!camReady) {
    reply(503, "ERROR=camera not ready err=0x" +
          String((unsigned)camErr, HEX) + "\n");
    return;
  }
  unsigned long t0 = millis();
  camera_fb_t* d = esp_camera_fb_get();
  if (!d) {
    reply(503, "ERROR=grab failed\n");
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
  Serial.printf("[/bmp] %dx%d in %lu ms (grab %lu ms, send %lu ms)\n",
                w, h, bmpTotalMs, bmpGrabMs, bmpSendMs);
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
  server.on("/bench", HTTP_GET, handleBench);
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
