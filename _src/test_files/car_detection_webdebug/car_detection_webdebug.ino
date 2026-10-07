/* Includes ---------------------------------------------------------------- */
#include <car_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------------------------
// USER SETTINGS
// ---------------------------------------------------------------------------
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM

#define AP_SSID        "CarDebug"
#define AP_PASSWORD    "12345678"      // min 8 chars
#define AP_CHANNEL     1
#define AP_MAX_CLIENTS 2               // keep low: saves RAM + airtime

// How the model input is made from the 320x240 frame.
// 1 = centre-crop to the model's aspect ratio, then scale (default, matches the
//     crop_and_interpolate call below). 0 = squash the whole frame.
// If boxes look shifted/stretched in the web view, flip this value.
#define RESIZE_MODE_CROP 1

// Extra runtime filter on top of the threshold baked into the Edge Impulse model.
// Adjustable live from the web page.
static float g_threshold = 0.50f;

// Areas to IGNORE, in 320x240 frame pixels: {x, y, w, h}. A detection whose
// centre falls inside a zone is shown in orange and NOT counted as a car.
// Use the click-coordinates readout on the web page to find these values.
struct Zone { int16_t x, y, w, h; };
static const Zone IGNORE_ZONES[] = {
    // {200, 0, 120, 60},
    {0, 0, 0, 0}   // placeholder so the array is never empty; w=0 zones are skipped
};

// ---------------------------------------------------------------------------
// Camera pins
// ---------------------------------------------------------------------------
#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
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

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

#define JPG_MAX_BYTES     (32 * 1024)   // QVGA JPEG is typically 8-15 KB
#define MAX_BOXES         10
#define LOG_LINES         24
#define LOG_LINE_LEN      96

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;   // allocated ONCE in setup (RGB888 320x240)

// --- web debug state (all touched only from the main loop => no locking) ----
static WebServer server(80);

static uint8_t *g_jpg = nullptr;          // copy of the JPEG of the last inferred frame
static size_t   g_jpg_len = 0;
static uint32_t g_seq = 0;                // increments after each published frame

struct Box { char label[16]; float value; int16_t x, y, w, h; bool ignored; };
static Box      g_boxes[MAX_BOXES];
static uint8_t  g_box_count = 0;
static uint32_t g_dsp_ms = 0, g_cls_ms = 0;

static char     g_log[LOG_LINES][LOG_LINE_LEN];
static uint32_t g_log_total = 0;          // total lines ever written
static uint8_t  g_last_car_count = 255;   // used to avoid flooding the log

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/* ---------------------------------------------------------------------------
 * Logging: prints to Serial AND keeps the last LOG_LINES lines for the web UI
 * ------------------------------------------------------------------------- */
static void logf(const char *fmt, ...) {
    char line[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    Serial.println(line);
    strlcpy(g_log[g_log_total % LOG_LINES], line, LOG_LINE_LEN);
    g_log_total++;
}

/* ---------------------------------------------------------------------------
 * Map a box from model-input coordinates to 320x240 frame coordinates
 * ------------------------------------------------------------------------- */
static void map_box_to_frame(const ei_impulse_result_bounding_box_t &bb,
                             int &x, int &y, int &w, int &h) {
    const float FW = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    const float FH = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    const float IW = EI_CLASSIFIER_INPUT_WIDTH;
    const float IH = EI_CLASSIFIER_INPUT_HEIGHT;
#if RESIZE_MODE_CROP
    float inAsp = IW / IH, frAsp = FW / FH;
    float cropW = FW, cropH = FH;
    if (frAsp > inAsp) cropW = FH * inAsp; else cropH = FW / inAsp;
    float offx = (FW - cropW) / 2.0f, offy = (FH - cropH) / 2.0f;
    float s = cropW / IW;
    x = (int)(offx + bb.x * s);
    y = (int)(offy + bb.y * s);
    w = (int)(bb.width * s);
    h = (int)(bb.height * s);
#else
    float sx = FW / IW, sy = FH / IH;
    x = (int)(bb.x * sx);
    y = (int)(bb.y * sy);
    w = (int)(bb.width * sx);
    h = (int)(bb.height * sy);
#endif
}

static bool in_ignore_zone(int cx, int cy) {
    for (size_t i = 0; i < sizeof(IGNORE_ZONES) / sizeof(IGNORE_ZONES[0]); i++) {
        const Zone &z = IGNORE_ZONES[i];
        if (z.w > 0 && cx >= z.x && cx < z.x + z.w && cy >= z.y && cy < z.y + z.h) return true;
    }
    return false;
}

/* ---------------------------------------------------------------------------
 * Web page (kept tiny; the BROWSER draws the boxes, not the ESP32)
 * ------------------------------------------------------------------------- */
static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Car Debug</title>
<style>
body{font-family:sans-serif;margin:8px;background:#111;color:#ddd}
canvas{width:100%;max-width:640px;background:#000;display:block}
pre{background:#000;color:#8f8;padding:6px;height:170px;overflow:auto;font-size:12px}
small{color:#999}
</style></head><body>
<canvas id="c" width="320" height="240"></canvas>
<div><small id="info">waiting...</small></div>
<div>Min confidence: <input id="th" type="range" min="0.1" max="0.99" step="0.01" value="0.5">
<span id="thv">0.50</span></div>
<small>Green = car. Orange = ignored by zone. Grey = ignore zones. Tap the image to read x,y.</small>
<div><small id="click"></small></div>
<pre id="log"></pre>
<script>
var cv=document.getElementById('c'),ctx=cv.getContext('2d'),img=new Image(),seq=-1,lastLog=0;
var th=document.getElementById('th'),thv=document.getElementById('thv');
th.onchange=function(){thv.textContent=th.value;fetch('/set?th='+th.value).catch(function(){});};
th.oninput=function(){thv.textContent=th.value;};
cv.onclick=function(e){var r=cv.getBoundingClientRect();
 var x=Math.round((e.clientX-r.left)*cv.width/r.width),y=Math.round((e.clientY-r.top)*cv.height/r.height);
 document.getElementById('click').textContent='clicked frame pixel x='+x+' y='+y;};
function draw(s){
 ctx.drawImage(img,0,0,cv.width,cv.height);
 ctx.fillStyle='rgba(128,128,128,0.35)';
 (s.zones||[]).forEach(function(z){ctx.fillRect(z[0],z[1],z[2],z[3]);});
 ctx.lineWidth=2;ctx.font='12px sans-serif';
 (s.b||[]).forEach(function(b){
  ctx.strokeStyle=b[6]?'#f90':'#0f0';ctx.fillStyle=ctx.strokeStyle;
  ctx.strokeRect(b[2],b[3],b[4],b[5]);
  ctx.fillText(b[0]+' '+b[1].toFixed(2),b[2]+2,b[3]>12?b[3]-3:b[3]+12);});
 document.getElementById('info').textContent='frame #'+s.seq+'  dsp '+s.dsp+' ms  cls '+s.cls+' ms  model input '+s.iw+'x'+s.ih;
}
function tick(){
 fetch('/state').then(function(r){return r.json();}).then(function(s){
  if(s.seq===seq){return;}
  seq=s.seq;
  img.onload=function(){draw(s);};
  img.src='/frame.jpg?s='+seq;
  return fetch('/log').then(function(r){return r.text();}).then(function(t){
   var p=document.getElementById('log');p.textContent=t;p.scrollTop=p.scrollHeight;});
 }).catch(function(){}).then(function(){setTimeout(tick,600);});
}
tick();
</script></body></html>)HTML";

/* --- handlers ------------------------------------------------------------ */
static void handle_index() {
    server.send_P(200, "text/html", INDEX_HTML);
}

static void handle_frame() {
    if (!g_jpg || g_jpg_len == 0) { server.send(404, "text/plain", "no frame yet"); return; }
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "image/jpeg", (const char *)g_jpg, g_jpg_len);
}

static void handle_state() {
    static char buf[1400];
    size_t n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{\"seq\":%lu,\"dsp\":%lu,\"cls\":%lu,\"iw\":%d,\"ih\":%d,\"zones\":[",
        (unsigned long)g_seq, (unsigned long)g_dsp_ms, (unsigned long)g_cls_ms,
        EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    bool first = true;
    for (size_t i = 0; i < sizeof(IGNORE_ZONES) / sizeof(IGNORE_ZONES[0]); i++) {
        const Zone &z = IGNORE_ZONES[i];
        if (z.w <= 0) continue;
        n += snprintf(buf + n, sizeof(buf) - n, "%s[%d,%d,%d,%d]", first ? "" : ",", z.x, z.y, z.w, z.h);
        first = false;
    }
    n += snprintf(buf + n, sizeof(buf) - n, "],\"b\":[");
    for (uint8_t i = 0; i < g_box_count && n < sizeof(buf) - 120; i++) {
        const Box &b = g_boxes[i];
        n += snprintf(buf + n, sizeof(buf) - n, "%s[\"%s\",%.3f,%d,%d,%d,%d,%d]",
                      i ? "," : "", b.label, b.value, b.x, b.y, b.w, b.h, b.ignored ? 1 : 0);
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
}

static void handle_log() {
    static char buf[LOG_LINES * LOG_LINE_LEN];
    size_t n = 0;
    uint32_t count = g_log_total < LOG_LINES ? g_log_total : LOG_LINES;
    uint32_t start = g_log_total - count;
    for (uint32_t i = 0; i < count; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s\n", g_log[(start + i) % LOG_LINES]);
    }
    buf[n < sizeof(buf) ? n : sizeof(buf) - 1] = 0;
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", buf);
}

static void handle_set() {
    if (server.hasArg("th")) {
        float v = server.arg("th").toFloat();
        if (v >= 0.05f && v <= 0.99f) {
            g_threshold = v;
            logf("Threshold set to %.2f", v);
        }
    }
    server.send(200, "text/plain", "ok");
}

static void web_init() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    WiFi.setSleep(false);  // SoftAP + sleep can make the page feel laggy
    server.on("/", handle_index);
    server.on("/frame.jpg", handle_frame);
    server.on("/state", handle_state);
    server.on("/log", handle_log);
    server.on("/set", handle_set);
    server.begin();
    Serial.printf("SoftAP '%s' up. Open http://%s/\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    // No "while (!Serial)" here: when you run from a power bank in the field
    // with no USB attached, that would stall startup.
    delay(200);
    Serial.println("Edge Impulse Inferencing Demo + SoftAP debug view");

    // Allocate the big buffers ONCE (avoids malloc/free churn + heap fragmentation)
    size_t snap_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE;
    snapshot_buf = (uint8_t *)(psramFound() ? ps_malloc(snap_size) : malloc(snap_size));
    g_jpg = (uint8_t *)(psramFound() ? ps_malloc(JPG_MAX_BYTES) : malloc(JPG_MAX_BYTES));
    if (!snapshot_buf || !g_jpg) {
        Serial.println("ERR: buffer allocation failed (is PSRAM enabled in Tools menu?)");
        while (true) delay(1000);
    }

    if (ei_camera_init() == false) {
        logf("Failed to initialize Camera!");
    } else {
        logf("Camera initialized");
    }

    web_init();
    logf("Starting continuous inference in 2 seconds...");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*/
void loop()
{
    // Serve the web UI between inferences. The server runs in this same thread,
    // so the JPEG/box data can never change while a request is being answered.
    server.handleClient();

    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        logf("Failed to capture image");
        return;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        logf("ERR: Failed to run classifier (%d)", err);
        return;
    }

    g_dsp_ms = result.timing.dsp;
    g_cls_ms = result.timing.classification;
    g_box_count = 0;
    uint8_t car_count = 0;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0 || bb.value < g_threshold) {
            continue;
        }
        if (g_box_count >= MAX_BOXES) break;

        int x, y, w, h;
        map_box_to_frame(bb, x, y, w, h);
        bool ign = in_ignore_zone(x + w / 2, y + h / 2);

        Box &b = g_boxes[g_box_count++];
        strlcpy(b.label, bb.label, sizeof(b.label));
        b.value = bb.value;
        b.x = x; b.y = y; b.w = w; b.h = h;
        b.ignored = ign;
        if (!ign) car_count++;

        logf("#%lu %s %.2f @ x=%d y=%d w=%d h=%d%s", (unsigned long)(g_seq + 1),
             bb.label, bb.value, x, y, w, h, ign ? " [IGNORED ZONE]" : "");
    }
#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        logf("%s: %.5f", ei_classifier_inferencing_categories[i], result.classification[i].value);
    }
#endif

    // Log "clear" only when the car count changes, so the log doesn't flood
    if (car_count == 0 && g_last_car_count != 0) {
        logf("#%lu no cars (dsp %lu ms, cls %lu ms)", (unsigned long)(g_seq + 1),
             (unsigned long)g_dsp_ms, (unsigned long)g_cls_ms);
    }
    g_last_car_count = car_count;

#if EI_CLASSIFIER_HAS_ANOMALY
    logf("Anomaly prediction: %.3f", result.anomaly);
#endif

    // Publish: boxes + matching JPEG become visible together
    g_seq++;

    // Let the browser fetch before we spend time on the next frame
    server.handleClient();
}

/**
 * @brief   Setup image sensor & start streaming
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
}

/**
 * @brief      Capture, rescale and crop image. Also keeps a copy of the JPEG
 *             so the web page shows exactly the frame that was analysed.
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // Keep a copy of the JPEG for the web view (skipped if unexpectedly large)
    if (fb->len <= JPG_MAX_BYTES) {
        memcpy(g_jpg, fb->buf, fb->len);
        g_jpg_len = fb->len;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
