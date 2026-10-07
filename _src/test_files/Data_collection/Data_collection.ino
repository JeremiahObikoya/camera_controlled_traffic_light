/*
  ESP32-CAM Dataset Collector
  ---------------------------
  Purpose: Collect a labeled-later JPEG image dataset from an AI-Thinker
  ESP32-CAM, for training a model in Edge Impulse that will eventually be
  flashed back onto this same board.

  Web UI features:
    - Live MJPEG video stream
    - Capture one image
    - Start/Stop continuous capture at a safe, adjustable rate
    - Clear all images
  - Store captured images in the browser and download them together

  IMPORTANT ARDUINO IDE SETTINGS:
    - Board: any ESP32 board definition that matches AI-Thinker ESP32-CAM
              (commonly "AI Thinker ESP32-CAM" or "ESP32 Wrover Module")
    - PSRAM: Enabled (if your board has it - most AI-Thinker boards do)
    - Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
        -> This is what gives LittleFS enough room to store a real dataset.
           Without this, you'll only fit a handful of images.
    - No extra libraries needed beyond the ESP32 board package
      (esp32 camera driver + LittleFS are both bundled with ESP32 core >=2.0.0)

  STORAGE NOTE:
    Captured images are returned to the browser and stored in IndexedDB,
    so the ESP32's internal flash is not used for the dataset.

  WORKFLOW FOR EDGE IMPULSE:
    1. Collect images using this sketch.
    2. Click "Download All (ZIP)" and unzip on your computer.
    3. Sort/rename into class folders as needed (this sketch does not label
       images - it just collects raw frames).
    4. Upload to Edge Impulse Studio (Data Acquisition -> Upload existing data).

  ROI (Region of Interest) - PLACEHOLDER:
    This sketch captures full frames as JPEG for simplicity, speed and low
    memory use. If you later want to crop to a Region of Interest before
    saving:
      - Switch config.pixel_format to PIXFORMAT_RGB565 in initCamera().
      - In captureAndSaveImage(), instead of writing fb->buf directly,
        treat fb->buf as a rowed RGB565 buffer (like the fb_data_t used in
        the original CameraWebServer face-detection code), copy out the
        sub-rectangle you want (roiX, roiY, roiW, roiH - defined below but
        unused for now), and pass that cropped buffer to fmt2jpg() from
        "img_converters.h" to get a JPEG buffer to save.
      - This is heavier on CPU/RAM than plain JPEG passthrough, which is
        why it isn't enabled by default here.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>

// =====================================================================
// Camera pin map - AI-Thinker ESP32-CAM
// =====================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM       4

// =====================================================================
// WiFi credentials
// =====================================================================
const char* ssid     = "JesusSaves";
const char* password = "banana2003";

// =====================================================================
// Configuration
// =====================================================================
#define MIN_INTERVAL_MS      300     // fastest allowed continuous-capture rate
#define DEFAULT_INTERVAL_MS  1000    // default continuous-capture rate
#define MIN_FREE_BYTES       15000   // stop capturing below this much free flash
#define MAX_ZIP_ENTRIES      300     // max files the ZIP export will include

// ROI placeholders (not active yet - see comment block at top of file)
int roiX = 0, roiY = 0, roiW = 160, roiH = 120;
bool roiEnabled = false;

// =====================================================================
// Globals
// =====================================================================
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

SemaphoreHandle_t camMutex;
volatile bool continuousActive = false;
volatile bool flashEnabled = false;
volatile int  captureIntervalMs = DEFAULT_INTERVAL_MS;
unsigned long lastCaptureMs = 0;
unsigned long imgCounter = 1;

static uint32_t crcTable[256];

struct ZipEntry {
  char name[40];
  uint32_t crc;
  uint32_t size;
  uint32_t offset;
};
static ZipEntry zipEntries[MAX_ZIP_ENTRIES];
static int zipEntryCount = 0;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// =====================================================================
// Forward declarations
// =====================================================================
bool initCamera();
void initImageCounter();
bool captureAndSaveImage(String &savedName);
void startCameraServer();

static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t capture_handler(httpd_req_t *req);
static esp_err_t start_handler(httpd_req_t *req);
static esp_err_t stop_handler(httpd_req_t *req);
static esp_err_t clear_handler(httpd_req_t *req);
static esp_err_t list_handler(httpd_req_t *req);
static esp_err_t img_handler(httpd_req_t *req);
static esp_err_t download_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t flash_handler(httpd_req_t *req);

// =====================================================================
// CRC32 (needed for building the ZIP file on the fly)
// =====================================================================
void buildCrcTable() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int j = 0; j < 8; j++) {
      c = (c & 1) ? (0xEDB88320 ^ (c >> 1)) : (c >> 1);
    }
    crcTable[i] = c;
  }
}

static inline uint32_t crc32_begin() { return 0xFFFFFFFF; }

static inline uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc = crcTable[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

static inline uint32_t crc32_finish(uint32_t crc) { return crc ^ 0xFFFFFFFF; }

static inline void writeLE16(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
}

static inline void writeLE32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

// =====================================================================
// Web page (served at "/")
// =====================================================================
static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-CAM Dataset Collector</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:#111;color:#eee;margin:0;padding:20px;}
  h1{font-size:1.3em;}
  .row{display:flex;flex-wrap:wrap;gap:20px;}
  .col{flex:1;min-width:300px;}
  img#stream{width:100%;max-width:480px;border:2px solid #444;border-radius:6px;background:#000;}
  button{background:#2b7a78;color:#fff;border:none;padding:10px 16px;margin:4px 4px 4px 0;border-radius:5px;cursor:pointer;font-size:0.95em;}
  button:hover{background:#3aa39f;}
  button:disabled{background:#555;cursor:not-allowed;}
  button.danger{background:#a83232;}
  button.danger:hover{background:#c94040;}
  input[type=number]{width:80px;padding:6px;border-radius:4px;border:1px solid #444;background:#222;color:#eee;}
  table{width:100%;border-collapse:collapse;margin-top:10px;font-size:0.85em;}
  th,td{padding:6px 8px;border-bottom:1px solid #333;text-align:left;}
  #status{margin:10px 0;padding:10px;background:#1b1b1b;border-radius:6px;font-size:0.9em;}
  .bar{background:#333;border-radius:4px;height:10px;overflow:hidden;margin-top:4px;}
  .bar-fill{background:#2b7a78;height:100%;}
  a.imglink{color:#7fd1cf;text-decoration:none;}
  a.imglink:hover{text-decoration:underline;}
  small{color:#999;}
</style>
</head>
<body>
<h1>ESP32-CAM Dataset Collector</h1>
<div class="row">
  <div class="col">
    <img id="stream" src="">
    <div id="status">Loading status...</div>
  </div>
  <div class="col">
    <div>
      <button id="btnCapture">Capture One</button>
      <button id="btnFlash">Flash: OFF</button>
      <br>
      Interval (ms): <input type="number" id="interval" value="1000" min="300" step="100">
      <button id="btnStart">Start Continuous</button>
      <button id="btnStop" disabled>Stop</button>
      <br>
      <button class="danger" id="btnClear">Clear Browser Images</button>
      <button id="btnDownload">Download All (ZIP)</button>
      <br>
    <small>Images are stored in this browser. Keep this page open while capturing.</small>
    </div>
    <table id="fileTable">
      <thead><tr><th>#</th><th>Filename</th><th>Size (KB)</th></tr></thead>
      <tbody id="fileBody"></tbody>
    </table>
  </div>
</div>

<script>
const streamImg = document.getElementById('stream');
streamImg.src = 'http://' + window.location.hostname + ':81/stream';

const btnCapture = document.getElementById('btnCapture');
const btnStart = document.getElementById('btnStart');
const btnStop = document.getElementById('btnStop');
const btnClear = document.getElementById('btnClear');
const btnDownload = document.getElementById('btnDownload');
const btnFlash = document.getElementById('btnFlash');
const intervalInput = document.getElementById('interval');
const statusDiv = document.getElementById('status');
const fileBody = document.getElementById('fileBody');

const dbPromise = new Promise((resolve, reject) => {
  const request = indexedDB.open('esp32-dataset', 1);
  request.onupgradeneeded = () => request.result.createObjectStore('images', {keyPath:'name'});
  request.onsuccess = () => resolve(request.result);
  request.onerror = () => reject(request.error);
});
async function getImages() {
  const db = await dbPromise;
  return new Promise((resolve, reject) => {
    const request = db.transaction('images').objectStore('images').getAll();
    request.onsuccess = () => resolve(request.result.sort((a,b) => a.name.localeCompare(b.name)));
    request.onerror = () => reject(request.error);
  });
}
async function refreshList() {
  const files = await getImages();
  const used = files.reduce((sum, f) => sum + f.blob.size, 0);
  statusDiv.innerHTML = 'Images: ' + files.length + ' | Browser storage: ' + (used/1024).toFixed(1) + ' KB' +
    ' | Continuous: ' + (continuousActive ? 'ON' : 'OFF');
  fileBody.innerHTML = '';
  files.forEach((f, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>' + (i+1) + '</td><td>' + f.name + '</td><td>' + (f.blob.size/1024).toFixed(1) + '</td>';
      fileBody.appendChild(tr);
  });
}
let continuousActive = false, captureTimer;
async function capture() {
  const response = await fetch('/capture');
  if (!response.ok) throw new Error('Capture failed');
  const blob = await response.blob();
  const name = 'img_' + String(Date.now()) + '.jpg';
  const db = await dbPromise;
  await new Promise((resolve, reject) => { const tx = db.transaction('images','readwrite'); tx.objectStore('images').put({name, blob}); tx.oncomplete=resolve; tx.onerror=()=>reject(tx.error); });
  refreshList();
}
btnCapture.onclick = () => capture().catch(e => alert(e.message));
btnStart.onclick = () => { continuousActive = true; const iv = Math.max(300, Number(intervalInput.value)||1000); capture(); captureTimer=setInterval(capture,iv); btnStart.disabled=true; btnStop.disabled=false; refreshList(); };
btnStop.onclick = () => { continuousActive=false; clearInterval(captureTimer); btnStart.disabled=false; btnStop.disabled=true; refreshList(); };

btnClear.onclick = () => {
  if (!confirm('Delete ALL images from this browser? This cannot be undone.')) return;
  dbPromise.then(db => new Promise(resolve => { const tx=db.transaction('images','readwrite'); tx.objectStore('images').clear(); tx.oncomplete=resolve; })).then(refreshList);
};
btnDownload.onclick = async () => { const files=await getImages(); if (!files.length) return alert('No images captured'); files.forEach((f,i) => { const a=document.createElement('a'); a.href=URL.createObjectURL(f.blob); a.download=f.name; a.click(); setTimeout(()=>URL.revokeObjectURL(a.href),1000+i*20); }); };
btnFlash.onclick = async () => { const on=btnFlash.textContent.endsWith('OFF'); await fetch('/flash?state='+(on?'on':'off')); btnFlash.textContent='Flash: '+(on?'ON':'OFF'); };

refreshList();
setInterval(refreshList, 3000);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// Camera init
// =====================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  // Dataset-friendly, MCU-friendly settings:
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QQVGA;              // 160x120
  config.jpeg_quality  = 10;                          // lower = better quality/larger file
  config.fb_count      = psramFound() ? 2 : 1;
  config.fb_location   = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;       // safer for buffer shared with mutex

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  return true;
}

// =====================================================================
// Determine starting counter value from existing files (so names stay unique
// across reboots without overwriting anything)
// =====================================================================
void initImageCounter() {
  imgCounter = 1;
  File root = LittleFS.open("/");
  if (!root) return;
  File file = root.openNextFile();
  while (file) {
    String fname = String(file.name());
    if (fname.startsWith("/")) fname.remove(0, 1);
    if (fname.startsWith("img_") && fname.endsWith(".jpg") && fname.length() > 8) {
      long num = fname.substring(4, fname.length() - 4).toInt();
      if (num >= (long)imgCounter) imgCounter = num + 1;
    }
    file = root.openNextFile();
  }
  root.close();
}

// =====================================================================
// Capture one frame and save it with a unique filename
// =====================================================================
bool captureAndSaveImage(String &savedName) {
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < MIN_FREE_BYTES) {
    Serial.println("Low storage - capture aborted");
    return false;
  }

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println("Camera busy - capture skipped");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    Serial.println("Camera capture failed");
    return false;
  }

  char filename[32];
  snprintf(filename, sizeof(filename), "/img_%05lu.jpg", imgCounter);

  bool ok = false;
  File f = LittleFS.open(filename, FILE_WRITE);
  if (f) {
    size_t written = f.write(fb->buf, fb->len);
    f.close();
    ok = (written == fb->len);
  }

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  if (ok) {
    imgCounter++;
    savedName = String(filename).substring(1); // drop leading '/'
  }
  return ok;
}

// =====================================================================
// HTTP handlers
// =====================================================================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t capture_handler(httpd_req_t *req) {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_FAIL;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { xSemaphoreGive(camMutex); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t result = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
  return result;
}

static esp_err_t flash_handler(httpd_req_t *req) {
  char query[32], state[8] = {0};
  if (httpd_req_get_url_query_len(req) > 0 && httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    httpd_query_key_value(query, "state", state, sizeof(state));
  flashEnabled = (strcmp(state, "on") == 0);
  digitalWrite(LED_GPIO_NUM, flashEnabled ? HIGH : LOW);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, flashEnabled ? "{\"flash\":true}" : "{\"flash\":false}");
}

static esp_err_t start_handler(httpd_req_t *req) {
  int interval = DEFAULT_INTERVAL_MS;
  char query[64];
  if (httpd_req_get_url_query_len(req) > 0 &&
      httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val[16];
    if (httpd_query_key_value(query, "interval", val, sizeof(val)) == ESP_OK) {
      interval = atoi(val);
    }
  }
  if (interval < MIN_INTERVAL_MS) interval = MIN_INTERVAL_MS;

  captureIntervalMs = interval;
  lastCaptureMs = millis() - interval; // trigger a capture almost immediately
  continuousActive = true;

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char resp[64];
  snprintf(resp, sizeof(resp), "{\"status\":\"started\",\"interval\":%d}", interval);
  return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t stop_handler(httpd_req_t *req) {
  continuousActive = false;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  const char *resp = "{\"status\":\"stopped\"}";
  return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t clear_handler(httpd_req_t *req) {
  continuousActive = false;

  std::vector<String> toDelete;
  File root = LittleFS.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      String fname = String(file.name());
      file.close();
      if (!fname.startsWith("/")) fname = "/" + fname;
      if (fname.indexOf("img_") >= 0 && fname.endsWith(".jpg")) {
        toDelete.push_back(fname);
      }
      file = root.openNextFile();
    }
    root.close();
  }

  int deleted = 0;
  for (size_t i = 0; i < toDelete.size(); i++) {
    if (LittleFS.remove(toDelete[i])) deleted++;
  }
  imgCounter = 1;

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char resp[64];
  snprintf(resp, sizeof(resp), "{\"status\":\"cleared\",\"deleted\":%d}", deleted);
  return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t list_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  String json = "{";
  json += "\"usedBytes\":" + String((uint32_t)LittleFS.usedBytes()) + ",";
  json += "\"totalBytes\":" + String((uint32_t)LittleFS.totalBytes()) + ",";
  json += "\"continuous\":";
  json += continuousActive ? "true" : "false";
  json += ",\"files\":[";

  int count = 0;
  File root = LittleFS.open("/");
  if (root) {
    File file = root.openNextFile();
    bool first = true;
    while (file) {
      String fname = String(file.name());
      if (fname.startsWith("/")) fname.remove(0, 1);
      if (fname.startsWith("img_") && fname.endsWith(".jpg")) {
        if (!first) json += ",";
        json += "{\"name\":\"" + fname + "\",\"size\":" + String((uint32_t)file.size()) + "}";
        first = false;
        count++;
      }
      file = root.openNextFile();
    }
    root.close();
  }

  json += "],\"count\":" + String(count) + "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t img_handler(httpd_req_t *req) {
  char query[64];
  char name[40] = {0};
  if (httpd_req_get_url_query_len(req) > 0 &&
      httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "name", name, sizeof(name));
  }
  if (strlen(name) == 0) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  String path = "/" + String(name);
  if (!LittleFS.exists(path)) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  File f = LittleFS.open(path, FILE_READ);
  httpd_resp_set_type(req, "image/jpeg");
  uint8_t buf[1024];
  int n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    httpd_resp_send_chunk(req, (const char*)buf, n);
  }
  f.close();
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// Streams a real ZIP file (stored/uncompressed entries) built on the fly.
static esp_err_t download_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/zip");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=dataset.zip");

  zipEntryCount = 0;
  uint32_t currentOffset = 0;

  uint8_t hdrBuf[40];
  uint8_t readBuf[1024];

  File root = LittleFS.open("/");
  File file = root ? root.openNextFile() : File();

  while (file) {
    String fname = String(file.name());
    if (fname.startsWith("/")) fname.remove(0, 1);

    if (fname.startsWith("img_") && fname.endsWith(".jpg") && zipEntryCount < MAX_ZIP_ENTRIES) {
      // ---- local file header ----
      int pos = 0;
      writeLE32(hdrBuf + pos, 0x04034b50); pos += 4;
      writeLE16(hdrBuf + pos, 20);         pos += 2;   // version needed
      writeLE16(hdrBuf + pos, 0x0008);     pos += 2;   // flag: data descriptor follows
      writeLE16(hdrBuf + pos, 0);          pos += 2;   // compression: stored
      writeLE16(hdrBuf + pos, 0);          pos += 2;   // mod time
      writeLE16(hdrBuf + pos, 0x21);       pos += 2;   // mod date (dummy but valid)
      writeLE32(hdrBuf + pos, 0);          pos += 4;   // crc32 (in descriptor instead)
      writeLE32(hdrBuf + pos, 0);          pos += 4;   // compressed size (in descriptor)
      writeLE32(hdrBuf + pos, 0);          pos += 4;   // uncompressed size (in descriptor)
      writeLE16(hdrBuf + pos, fname.length()); pos += 2;
      writeLE16(hdrBuf + pos, 0);          pos += 2;   // extra field length

      httpd_resp_send_chunk(req, (const char*)hdrBuf, pos);
      httpd_resp_send_chunk(req, fname.c_str(), fname.length());

      uint32_t localHeaderOffset = currentOffset;
      currentOffset += pos + fname.length();

      // ---- file data, streamed straight from flash ----
      uint32_t crc = crc32_begin();
      uint32_t totalRead = 0;
      int n;
      while ((n = file.read(readBuf, sizeof(readBuf))) > 0) {
        crc = crc32_update(crc, readBuf, n);
        httpd_resp_send_chunk(req, (const char*)readBuf, n);
        totalRead += n;
        currentOffset += n;
      }
      crc = crc32_finish(crc);

      // ---- data descriptor ----
      pos = 0;
      writeLE32(hdrBuf + pos, 0x08074b50); pos += 4;
      writeLE32(hdrBuf + pos, crc);        pos += 4;
      writeLE32(hdrBuf + pos, totalRead);  pos += 4;
      writeLE32(hdrBuf + pos, totalRead);  pos += 4;
      httpd_resp_send_chunk(req, (const char*)hdrBuf, pos);
      currentOffset += pos;

      // ---- remember for central directory ----
      ZipEntry &e = zipEntries[zipEntryCount++];
      strncpy(e.name, fname.c_str(), sizeof(e.name) - 1);
      e.name[sizeof(e.name) - 1] = 0;
      e.crc = crc;
      e.size = totalRead;
      e.offset = localHeaderOffset;
    }

    file.close();
    file = root.openNextFile();
  }
  if (root) root.close();

  // ---- central directory ----
  uint32_t cdStart = currentOffset;
  uint8_t cdBuf[70];

  for (int i = 0; i < zipEntryCount; i++) {
    ZipEntry &e = zipEntries[i];
    size_t nameLen = strlen(e.name);
    int pos = 0;
    writeLE32(cdBuf + pos, 0x02014b50); pos += 4;
    writeLE16(cdBuf + pos, 20);         pos += 2;   // version made by
    writeLE16(cdBuf + pos, 20);         pos += 2;   // version needed
    writeLE16(cdBuf + pos, 0x0008);     pos += 2;   // flags
    writeLE16(cdBuf + pos, 0);          pos += 2;   // compression
    writeLE16(cdBuf + pos, 0);          pos += 2;   // time
    writeLE16(cdBuf + pos, 0x21);       pos += 2;   // date
    writeLE32(cdBuf + pos, e.crc);      pos += 4;
    writeLE32(cdBuf + pos, e.size);     pos += 4;
    writeLE32(cdBuf + pos, e.size);     pos += 4;
    writeLE16(cdBuf + pos, nameLen);    pos += 2;
    writeLE16(cdBuf + pos, 0);          pos += 2;   // extra length
    writeLE16(cdBuf + pos, 0);          pos += 2;   // comment length
    writeLE16(cdBuf + pos, 0);          pos += 2;   // disk number
    writeLE16(cdBuf + pos, 0);          pos += 2;   // internal attrs
    writeLE32(cdBuf + pos, 0);          pos += 4;   // external attrs
    writeLE32(cdBuf + pos, e.offset);   pos += 4;

    httpd_resp_send_chunk(req, (const char*)cdBuf, pos);
    httpd_resp_send_chunk(req, e.name, nameLen);
    currentOffset += pos + nameLen;
  }

  uint32_t cdSize = currentOffset - cdStart;

  // ---- end of central directory record ----
  uint8_t eocd[22];
  int pos = 0;
  writeLE32(eocd + pos, 0x06054b50);   pos += 4;
  writeLE16(eocd + pos, 0);            pos += 2;
  writeLE16(eocd + pos, 0);            pos += 2;
  writeLE16(eocd + pos, zipEntryCount); pos += 2;
  writeLE16(eocd + pos, zipEntryCount); pos += 2;
  writeLE32(eocd + pos, cdSize);       pos += 4;
  writeLE32(eocd + pos, cdStart);      pos += 4;
  writeLE16(eocd + pos, 0);            pos += 2;

  httpd_resp_send_chunk(req, (const char*)eocd, pos);
  httpd_resp_send_chunk(req, NULL, 0); // end response

  return ESP_OK;
}

// Live MJPEG stream
static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partBuf[64];

  while (true) {
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      res = ESP_FAIL;
      break;
    }
    camera_fb_t *fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) {
      Serial.println("Camera capture failed (stream)");
      res = ESP_FAIL;
      break;
    }

    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;
  }
  return res;
}

// =====================================================================
// Server setup
// =====================================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 12;
  config.stack_size = 8192;

  httpd_uri_t index_uri    = { "/",         HTTP_GET, index_handler,    NULL };
  httpd_uri_t capture_uri  = { "/capture",  HTTP_GET, capture_handler,  NULL };
  httpd_uri_t start_uri    = { "/start",    HTTP_GET, start_handler,    NULL };
  httpd_uri_t stop_uri     = { "/stop",     HTTP_GET, stop_handler,     NULL };
  httpd_uri_t clear_uri    = { "/clear",    HTTP_GET, clear_handler,    NULL };
  httpd_uri_t list_uri     = { "/list",     HTTP_GET, list_handler,     NULL };
  httpd_uri_t img_uri      = { "/img",      HTTP_GET, img_handler,      NULL };
  httpd_uri_t download_uri = { "/download", HTTP_GET, download_handler, NULL };
  httpd_uri_t flash_uri    = { "/flash",    HTTP_GET, flash_handler,    NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &start_uri);
    httpd_register_uri_handler(camera_httpd, &stop_uri);
    httpd_register_uri_handler(camera_httpd, &clear_uri);
    httpd_register_uri_handler(camera_httpd, &list_uri);
    httpd_register_uri_handler(camera_httpd, &img_uri);
    httpd_register_uri_handler(camera_httpd, &download_uri);
    httpd_register_uri_handler(camera_httpd, &flash_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// =====================================================================
// Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.printf("LittleFS mounted. Used: %u / Total: %u bytes\n",
                   (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  }

  buildCrcTable();
  initImageCounter();

  camMutex = xSemaphoreCreateMutex();

  if (!initCamera()) {
    Serial.println("Camera init failed - halting.");
    while (true) delay(1000);
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  startCameraServer();

  Serial.print("Open http://");
  Serial.print(WiFi.localIP());
  Serial.println("/ in your browser.");
}

void loop() {
  if (continuousActive) {
    unsigned long now = millis();
    if (now - lastCaptureMs >= (unsigned long)captureIntervalMs) {
      lastCaptureMs = now;
      String name;
      if (captureAndSaveImage(name)) {
        Serial.println("Captured: " + name);
      } else {
        Serial.println("Capture failed / storage full - stopping continuous mode.");
        continuousActive = false;
      }
    }
  }
  delay(20);
} 
