/* Includes ---------------------------------------------------------------- */
#include <car_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// ---------------------------------------------------------------------------
// USER SETTINGS
// ---------------------------------------------------------------------------
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM

#define AP_SSID        "CarDebug"
#define AP_PASSWORD    "12345678"      // min 8 chars
#define AP_CHANNEL     1
#define AP_MAX_CLIENTS 4               // traffic controller + a few phones/laptops

// Link to the traffic-light ESP32 (must match UDP_PORT in the controller sketch)
#define UDP_PORT              4210
#define CONTROLLER_TIMEOUT_MS 3000     // no heartbeat for this long = NOT CONNECTED

// Lane N on the traffic controller = the ROI with this name (set in roi_tool.html).
// Looked up by NAME, so the order of the ROI block no longer matters.
static const char *LANE_ROI_NAMES[3] = {"ROI 1", "ROI 2", "ROI 3"};

// How the model input is made from the 320x240 frame.
// 1 = centre-crop to the model's aspect ratio, then scale. 0 = squash whole frame.
// If boxes look shifted/stretched on the web page, flip this value.
#define RESIZE_MODE_CROP 1

// Which point of a detection decides whether it is "inside" a region.
// 1 = bottom-centre of the box, 0 = centre of the box.
// (For FOMO models, 0 is usually the better choice.)
#define ROI_ANCHOR_BOTTOM_CENTER 1

// Extra runtime filter on top of the threshold baked into the model.
// Adjustable live from the web page.
static float g_threshold = 0.55f;
#define TRAFFIC_MIN_AVERAGE_CONFIDENCE 0.80f
#define TRAFFIC_REQUIRED_FRAMES 3

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

#define JPG_MAX_BYTES     (32 * 1024)
#define MAX_BOXES         20    // raised from 10: splitting merged detections can produce more boxes
#define MAX_ROIS          8
#define LOG_LINES         40
#define LOG_LINE_LEN      96

// ---------------------------------------------------------------------------
// Maximum bounding-box for a single car.
// Measured from CAR_Size_PTS at the worst-case (diagonal) pose:
//   width  = 177 - 151 = 26 px
//   height = 154 - 124 = 30 px   →  area = 780 px²
// A 25 % tolerance is added for slight angles and lens distortion.
// Boxes whose area exceeds MAX_CAR_BOX_AREA are treated as merged detections
// and are split into virtual sub-detections for ROI counting.
// ---------------------------------------------------------------------------
#define MAX_CAR_BOX_W     26          // pixels
#define MAX_CAR_BOX_H     30          // pixels
#define MAX_CAR_BOX_AREA  975         // 780 * 1.25  (int32 arithmetic, safe)
#define MAX_CAR_SPLIT     4           // maximum cars to extract from one merged box

// Global flag: show oversized (likely merged) boxes on the web interface.
// They are ALWAYS excluded from the vehicle count regardless of this flag.
// Toggle live with  GET /set?oversized=1  or  GET /set?oversized=0
static bool g_show_oversized = false;

// ---------------------------------------------------------------------------
// REGIONS OF INTEREST (frame pixels, 320x240)
// Generate this block with roi_tool.html and paste it between the markers.
// ---------------------------------------------------------------------------
struct Roi { const char *name; const int16_t *pts; uint8_t n; };  // pts = x0,y0,x1,y1,...

// >>> ROI BLOCK START >>>
// Frame 320x240 px. Paste this over the ROI block in the sketch.
static const int16_t ROI_0_PTS[] = {117,110, 120,42, 151,43, 160,111};
static const int16_t ROI_1_PTS[] = {122,149, 120,186, 2,190, 3,156};
static const int16_t ROI_2_PTS[] = {187,149, 182,104, 257,102, 269,147};
static const Roi ROIS[] = {
    {"ROI 2", ROI_2_PTS, 4},
    {"ROI 3", ROI_0_PTS, 4},
    {"ROI 1", ROI_1_PTS, 4},
};
// <<< ROI BLOCK END <<<

#define ROI_COUNT (sizeof(ROIS) / sizeof(ROIS[0]))
static_assert(ROI_COUNT <= MAX_ROIS, "Too many ROIs (max 8)");

// ---------------------------------------------------------------------------
// Camera pins (defined in camera_pins.h)
// ---------------------------------------------------------------------------
#include "camera_pins.h"

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;   // allocated ONCE in setup (RGB888 320x240)

// --- web debug state (all touched only from the main loop => no locking) ----
static WebServer server(80);

static uint8_t *g_jpg = nullptr;          // JPEG of the last inferred frame
static size_t   g_jpg_len = 0;
static uint32_t g_seq = 0;

struct Box {
    char     label[16];
    float    value;
    int16_t  x, y, w, h;      // frame coordinates (320x240)
    int16_t  ax, ay;          // anchor point used for the ROI test
    uint8_t  mask;            // bit i set = inside ROI i
    bool     oversized;       // true → box area exceeds MAX_CAR_BOX_AREA (merged/noise)
    uint8_t  split_n;         // how many virtual cars were extracted (1 = normal)
};
static Box      g_boxes[MAX_BOXES];
static uint8_t  g_box_count = 0;
static uint8_t  g_roi_count[MAX_ROIS];
static uint8_t  g_traffic_count[MAX_ROIS];
static uint8_t  g_traffic_streak[MAX_ROIS];
static float    g_traffic_conf_sum[MAX_ROIS];
static float    g_traffic_conf_history[MAX_ROIS][TRAFFIC_REQUIRED_FRAMES];
static bool     g_camera_control_mode = true;   // true = CAMERA, false = TIME
static uint32_t g_dsp_ms = 0, g_cls_ms = 0;

// --- link to the traffic controller (UDP, independent of the web server) ----
static WiFiUDP   udp;
static IPAddress g_ctrl_ip;
static uint16_t  g_ctrl_port = UDP_PORT;
static bool      g_ctrl_seen = false;       // have we EVER heard a heartbeat
static bool      g_ctrl_up = false;         // heartbeat received recently
static uint32_t  g_ctrl_last_ms = 0;
static int       g_ctrl_lane = 0;           // lane the controller says is green (0 = unknown)
static bool      g_ctrl_mode_camera = true; // mode the controller says it is using

static int8_t    g_lane_roi[3] = {-1, -1, -1};  // lane -> index into ROIS[]

static char     g_log[LOG_LINES][LOG_LINE_LEN];
static uint32_t g_log_total = 0;

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

    .jpeg_quality = 25,            // was 12; ~40% smaller file = faster web delivery
    .fb_count = 2,                 // was 1; double-buffer prevents partial/torn frames
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,  // was GRAB_WHEN_EMPTY; always return newest frame
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/* ---------------------------------------------------------------------------
 * Output: prints to Serial (each call = one line, \r\n like the original
 * example) AND keeps the last LOG_LINES lines for the web page.
 * ------------------------------------------------------------------------- */
static void dbg_printf(const char *fmt, ...) {
    char line[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    Serial.print(line);
    Serial.print("\r\n");
    strlcpy(g_log[g_log_total % LOG_LINES], line, LOG_LINE_LEN);
    g_log_total++;
}

/* ---------------------------------------------------------------------------
 * Geometry helpers
 * ------------------------------------------------------------------------- */
// Map a box from model-input coordinates to 320x240 frame coordinates
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

// Ray-casting point-in-polygon. pts = x0,y0,x1,y1,... (n vertices)
static bool point_in_poly(int px, int py, const int16_t *pts, uint8_t n) {
    bool inside = false;
    for (uint8_t i = 0, j = n - 1; i < n; j = i++) {
        float xi = pts[2 * i], yi = pts[2 * i + 1];
        float xj = pts[2 * j], yj = pts[2 * j + 1];
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

/* ---------------------------------------------------------------------------
 * Traffic controller link (UDP)
 *   controller -> camera : "HB,<green lane>,<C|T>"        every 0.5 s (heartbeat)
 *   camera -> controller : "CAM,<seq>,<C|T>,<l1>,<l2>,<l3>" after every inference
 *                                                          and on every mode change
 * UDP is used on purpose: it never waits on the web server, so browser
 * traffic and inference time cannot make the controller time out.
 * ------------------------------------------------------------------------- */
static uint8_t lane_count(int lane) {      // lane 0..2
    int8_t r = g_lane_roi[lane];
    return (r >= 0) ? g_traffic_count[r] : 0;
}

static void send_to_controller() {
    if (!g_ctrl_seen) return;              // don't know where to send yet
    char msg[48];
    snprintf(msg, sizeof(msg), "CAM,%lu,%c,%u,%u,%u",
             (unsigned long)g_seq, g_camera_control_mode ? 'C' : 'T',
             (unsigned)lane_count(0), (unsigned)lane_count(1), (unsigned)lane_count(2));
    udp.beginPacket(g_ctrl_ip, g_ctrl_port);
    udp.print(msg);
    udp.endPacket();
}

static void poll_controller() {
    for (int i = 0; i < 8; i++) {          // drain whatever has arrived
        int sz = udp.parsePacket();
        if (sz <= 0) break;
        char buf[48];
        int n = udp.read(buf, sizeof(buf) - 1);
        if (n <= 0) continue;
        buf[n] = 0;
        int lane = 0;
        char m = 'C';
        if (sscanf(buf, "HB,%d,%c", &lane, &m) == 2) {
            g_ctrl_ip = udp.remoteIP();
            g_ctrl_port = udp.remotePort();
            g_ctrl_seen = true;
            g_ctrl_last_ms = millis();
            g_ctrl_lane = lane;
            g_ctrl_mode_camera = (m == 'C');
        }
    }
    bool up = g_ctrl_seen && (millis() - g_ctrl_last_ms < CONTROLLER_TIMEOUT_MS);
    if (up != g_ctrl_up) {
        g_ctrl_up = up;
        if (up) dbg_printf("Traffic controller CONNECTED (%s)", g_ctrl_ip.toString().c_str());
        else    dbg_printf("Traffic controller NOT CONNECTED (no heartbeat)");
    }
}

/* ---------------------------------------------------------------------------
 * Web page (the BROWSER draws boxes and regions, not the ESP32)
 * ------------------------------------------------------------------------- */
static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Car ROI Debug</title>
<style>
body{font-family:sans-serif;margin:8px;background:#111;color:#ddd}
canvas{width:100%;max-width:640px;background:#000;display:block}
pre{background:#000;color:#8f8;padding:6px;height:220px;overflow:auto;font-size:12px}
small{color:#999}
#dot{display:inline-block;width:12px;height:12px;border-radius:50%;background:#c00;margin-right:6px;vertical-align:middle}
#ctl{font-size:14px;color:#ddd}
#modeButton{margin:8px 4px 8px 0;padding:10px 14px;font-size:15px;border-radius:6px;border:1px solid #555;background:#333;color:#fff}
#ovButton{margin:8px 0;padding:10px 14px;font-size:15px;border-radius:6px;border:1px solid #555;background:#333;color:#f90}
</style></head><body>
<canvas id="c" width="320" height="240"></canvas>
<div><small id="info">waiting...</small></div>
<div style="margin-top:6px"><span id="dot"></span><span id="ctl">Traffic controller: checking...</span></div>
<button id="modeButton" onclick="toggleMode()">Control mode: ...</button>
<button id="ovButton"   onclick="toggleOversized()">Show oversized: OFF</button>
<div>Min confidence: <input id="th" type="range" min="0" max="0.99" step="0.01" value="0">
<span id="thv">0.00</span></div>
<small>Blue = regions. Green = valid car inside ROI. Grey = valid car outside ROI. <span style="color:#f90">Orange = oversized/merged (excluded from count)</span>. Tap image for x,y.</small>
<div><small id="click"></small></div>
<pre id="log"></pre>
<script>
var cv=document.getElementById('c'),ctx=cv.getContext('2d'),img=new Image(),seq=-1,rois=[],first=true,curMode='camera';
var th=document.getElementById('th'),thv=document.getElementById('thv');
var modeButton=document.getElementById('modeButton');
var ovButton=document.getElementById('ovButton');
var showOversized=false;
th.onchange=function(){fetch('/set?th='+th.value).catch(function(){});};
th.oninput=function(){thv.textContent=parseFloat(th.value).toFixed(2);};
function setStatus(s){
 curMode=s.mode;
 document.getElementById('dot').style.background=s.ctl?'#0c0':'#c00';
 document.getElementById('ctl').textContent=s.ctl
  ?'Traffic controller CONNECTED - green lane: '+(s.lane||'?')+' - controller using: '+s.cmode.toUpperCase()
  :'Traffic controller NOT CONNECTED';
 modeButton.textContent='Control mode: '+s.mode.toUpperCase()+'  (tap to switch to '+(s.mode==='camera'?'TIME':'CAMERA')+')';
 // Sync oversized toggle from server state (survives page reload)
 if(typeof s.showov!=='undefined'){
  showOversized=(s.showov===1);
  ovButton.textContent='Show oversized: '+(showOversized?'ON':'OFF');
 }
}
function refreshStatus(){fetch('/state').then(function(r){return r.json();}).then(setStatus).catch(function(){});}
function toggleMode(){
 var next=curMode==='camera'?'time':'camera';
 fetch('/mode?value='+next).then(refreshStatus).catch(function(){});
}
function toggleOversized(){
 showOversized=!showOversized;
 ovButton.textContent='Show oversized: '+(showOversized?'ON':'OFF');
 fetch('/set?oversized='+(showOversized?'1':'0')).catch(function(){});
}
cv.onclick=function(e){var r=cv.getBoundingClientRect();
 var x=Math.round((e.clientX-r.left)*cv.width/r.width),y=Math.round((e.clientY-r.top)*cv.height/r.height);
 document.getElementById('click').textContent='clicked frame pixel x='+x+' y='+y;};
fetch('/rois').then(function(r){return r.json();}).then(function(j){rois=j;}).catch(function(){});
function draw(s){
 ctx.drawImage(img,0,0,cv.width,cv.height);
 ctx.lineWidth=2;ctx.font='12px sans-serif';
 rois.forEach(function(r,i){
  ctx.beginPath();
  for(var k=0;k<r.p.length;k+=2){if(k){ctx.lineTo(r.p[k],r.p[k+1]);}else{ctx.moveTo(r.p[0],r.p[1]);}}
  ctx.closePath();ctx.fillStyle='rgba(0,150,255,0.22)';ctx.fill();
  ctx.strokeStyle='#09f';ctx.stroke();ctx.fillStyle='#6cf';
  ctx.fillText(r.n+': '+(s.c[i]||0),r.p[0]+4,r.p[1]>14?r.p[1]-4:r.p[1]+14);
 });
 (s.b||[]).forEach(function(b){
  var oversized=(b[9]===1);
  if(oversized&&!showOversized){return;}   // hide oversized boxes unless toggle is ON
  var inside=(b[6]!==0)&&!oversized;
  // Green = valid & inside ROI, Grey = valid & outside, Orange = oversized/merged
  ctx.strokeStyle=oversized?'#f90':(inside?'#0f0':'#999');
  ctx.fillStyle=ctx.strokeStyle;
  ctx.strokeRect(b[2],b[3],b[4],b[5]);
  ctx.fillText(b[0]+' '+b[1].toFixed(2),b[2]+2,b[3]>12?b[3]-3:b[3]+12);
  ctx.beginPath();ctx.arc(b[7],b[8],3,0,6.3);ctx.fill();
 });
 document.getElementById('info').textContent='frame #'+s.seq+'  dsp '+s.dsp+' ms  cls '+s.cls+' ms  model input '+s.iw+'x'+s.ih;
}
function tick(){
 fetch('/state').then(function(r){return r.json();}).then(function(s){
  setStatus(s);                       // status updates on EVERY poll, not only on new frames
  if(first){th.value=s.th;thv.textContent=s.th.toFixed(2);first=false;}
  if(s.seq===seq){return;}
  seq=s.seq;
  img.onload=function(){draw(s);};
  img.src='/frame.jpg?s='+seq;
  return fetch('/log').then(function(r){return r.text();}).then(function(t){
   var p=document.getElementById('log');p.textContent=t;p.scrollTop=p.scrollHeight;});
 }).catch(function(){}).then(function(){setTimeout(tick,1200);});  // 1200ms ≈ 2× inference time
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
    server.sendHeader("Content-Length", String(g_jpg_len));
    server.setContentLength(g_jpg_len);
    server.send(200, "image/jpeg", "");    // send headers only
    // Stream in 4 KB chunks so the TCP stack never stalls on one large write
    const size_t CHUNK = 4096;
    for (size_t off = 0; off < g_jpg_len; off += CHUNK) {
        size_t sz = g_jpg_len - off;
        if (sz > CHUNK) sz = CHUNK;
        server.sendContent((const char *)g_jpg + off, sz);
    }
}

static void handle_rois() {
    static char buf[1500];
    size_t n = 0;
    buf[n++] = '[';
    for (size_t r = 0; r < ROI_COUNT && n < sizeof(buf) - 60; r++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s{\"n\":\"%s\",\"p\":[", r ? "," : "", ROIS[r].name);
        for (uint8_t k = 0; k < ROIS[r].n * 2 && n < sizeof(buf) - 20; k++) {
            n += snprintf(buf + n, sizeof(buf) - n, "%s%d", k ? "," : "", ROIS[r].pts[k]);
        }
        n += snprintf(buf + n, sizeof(buf) - n, "]}");
    }
    if (n < sizeof(buf) - 2) buf[n++] = ']';
    buf[n] = 0;
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
}

static void handle_state() {
    static char buf[1600];
    size_t n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{\"seq\":%lu,\"dsp\":%lu,\"cls\":%lu,\"iw\":%d,\"ih\":%d,\"th\":%.2f,"
        "\"ctl\":%d,\"lane\":%d,\"cmode\":\"%s\",\"mode\":\"%s\","
        "\"showov\":%d,\"c\":[",
        (unsigned long)g_seq, (unsigned long)g_dsp_ms, (unsigned long)g_cls_ms,
        EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, g_threshold,
        g_ctrl_up ? 1 : 0, g_ctrl_lane,
        g_ctrl_mode_camera ? "camera" : "time",
        g_camera_control_mode ? "camera" : "time",
        g_show_oversized ? 1 : 0);
    for (size_t r = 0; r < ROI_COUNT; r++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s%u", r ? "," : "", (unsigned)g_roi_count[r]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "],\"b\":[");
    for (uint8_t i = 0; i < g_box_count && n < sizeof(buf) - 130; i++) {
        const Box &b = g_boxes[i];
        // Box JSON: [label, conf, x, y, w, h, mask, ax, ay, oversized]
        n += snprintf(buf + n, sizeof(buf) - n, "%s[\"%s\",%.3f,%d,%d,%d,%d,%u,%d,%d,%d]",
                      i ? "," : "", b.label, b.value, b.x, b.y, b.w, b.h,
                      (unsigned)b.mask, b.ax, b.ay, b.oversized ? 1 : 0);
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
}

// Read-only debug view of what is being sent to the controller (no side effects)
static void handle_traffic() {
    char buf[160];
    snprintf(buf, sizeof(buf), "{\"counts\":[%u,%u,%u],\"mode\":\"%s\",\"controller\":%s}",
             (unsigned)lane_count(0), (unsigned)lane_count(1), (unsigned)lane_count(2),
             g_camera_control_mode ? "camera" : "time", g_ctrl_up ? "true" : "false");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
}

static void handle_mode() {
    if (server.hasArg("value")) {
        String value = server.arg("value");
        bool newMode = g_camera_control_mode;
        if (value == "camera") newMode = true;
        else if (value == "time") newMode = false;
        if (newMode != g_camera_control_mode) {
            g_camera_control_mode = newMode;
            dbg_printf("*** MODE CHANGED -> %s CONTROL ***", newMode ? "CAMERA" : "TIME");
            send_to_controller();          // tell the controller immediately
        }
    }
    server.send(200, "text/plain", g_camera_control_mode ? "camera" : "time");
}

static void handle_log() {
    static char buf[LOG_LINES * LOG_LINE_LEN];
    size_t n = 0;
    uint32_t count = g_log_total < LOG_LINES ? g_log_total : LOG_LINES;
    uint32_t start = g_log_total - count;
    for (uint32_t i = 0; i < count && n < sizeof(buf) - LOG_LINE_LEN; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s\n", g_log[(start + i) % LOG_LINES]);
    }
    buf[n < sizeof(buf) ? n : sizeof(buf) - 1] = 0;
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", buf);
}

static void handle_set() {
    if (server.hasArg("th")) {
        float v = server.arg("th").toFloat();
        if (v >= 0.0f && v <= 0.99f) {
            g_threshold = v;
            dbg_printf("Threshold set to %.2f", v);
        }
    }
    if (server.hasArg("oversized")) {
        String ov = server.arg("oversized");
        g_show_oversized = (ov == "1" || ov == "true");
        dbg_printf("Show oversized boxes: %s", g_show_oversized ? "ON" : "OFF");
    }
    server.send(200, "text/plain", "ok");
}

static void web_init() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    WiFi.setSleep(false);
    server.on("/", handle_index);
    server.on("/frame.jpg", handle_frame);
    server.on("/rois", handle_rois);
    server.on("/state", handle_state);
    server.on("/log", handle_log);
    server.on("/set", handle_set);
    server.on("/traffic", handle_traffic);
    server.on("/mode", handle_mode);
    server.begin();
    udp.begin(UDP_PORT);
    Serial.printf("SoftAP '%s' up. Open http://%s/\r\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    // No "while (!Serial)": it would stall startup when running without USB.
    delay(200);
    Serial.println("Edge Impulse Inferencing Demo");

    // Allocate the big buffers ONCE (no malloc/free churn, no fragmentation)
    size_t snap_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE;
    snapshot_buf = (uint8_t *)(psramFound() ? ps_malloc(snap_size) : malloc(snap_size));
    g_jpg = (uint8_t *)(psramFound() ? ps_malloc(JPG_MAX_BYTES) : malloc(JPG_MAX_BYTES));
    if (!snapshot_buf || !g_jpg) {
        Serial.println("ERR: buffer allocation failed (is PSRAM enabled in the Tools menu?)");
        while (true) delay(1000);
    }

    if (ei_camera_init() == false) {
        dbg_printf("Failed to initialize Camera!");
    }
    else {
        dbg_printf("Camera initialized");
    }

    // Map lane 1..3 to ROIs by NAME so the order in the ROI block doesn't matter
    for (int l = 0; l < 3; l++) {
        g_lane_roi[l] = -1;
        for (size_t r = 0; r < ROI_COUNT; r++) {
            if (strcmp(ROIS[r].name, LANE_ROI_NAMES[l]) == 0) { g_lane_roi[l] = (int8_t)r; break; }
        }
        if (g_lane_roi[l] < 0) dbg_printf("WARNING: no ROI named '%s' - lane %d will report 0", LANE_ROI_NAMES[l], l + 1);
        else                   dbg_printf("Lane %d = %s (ROI index %d)", l + 1, LANE_ROI_NAMES[l], (int)g_lane_roi[l]);
    }

    web_init();

    dbg_printf("Starting continious inference in 2 seconds...");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*/
void loop()
{
    // Serve the web UI between inferences (same thread => no locking needed)
    server.handleClient();
    poll_controller();

    // Keepalive: re-send the last known counts so the controller never times
    // out during a long inference cycle (DSP + classification can take 2-5 s).
    static uint32_t lastKeepaliveMs = 0;
    if (millis() - lastKeepaliveMs >= 1500) {   // was 2000 — tighter heartbeat
        lastKeepaliveMs = millis();
        send_to_controller();          // sends last debounced counts; no-op until first HB received
    }

    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        dbg_printf("Failed to capture image");
        return;
    }

    // Serve any browser requests that arrived during capture, BEFORE we block
    // for ~500 ms inside the classifier.
    server.handleClient();

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        dbg_printf("ERR: Failed to run classifier (%d)", err);
        return;
    }

    g_dsp_ms = result.timing.dsp;
    g_cls_ms = result.timing.classification;
    g_box_count = 0;
    memset(g_roi_count, 0, sizeof(g_roi_count));
    float frame_conf_sum[MAX_ROIS] = {0};

    // print the predictions
    dbg_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): ",
               (int)result.timing.dsp, (int)result.timing.classification, (int)result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    dbg_printf("Object detection bounding boxes:");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0 || bb.value < g_threshold) {
            continue;
        }
        dbg_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]",
                   bb.label, bb.value,
                   (unsigned)bb.x, (unsigned)bb.y,
                   (unsigned)bb.width, (unsigned)bb.height);

        // Map model coordinates → 320x240 frame coordinates
        int x, y, w, h;
        map_box_to_frame(bb, x, y, w, h);

        // Display anchor (centre dot shown on the web page for the original box)
        int disp_ax = x + w / 2;
#if ROI_ANCHOR_BOTTOM_CENTER
        int disp_ay = y + h;
#else
        int disp_ay = y + h / 2;
#endif
        disp_ax = constrain(disp_ax, 0, EI_CAMERA_RAW_FRAME_BUFFER_COLS - 1);
        disp_ay = constrain(disp_ay, 0, EI_CAMERA_RAW_FRAME_BUFFER_ROWS - 1);

        // ----------------------------------------------------------------
        // Oversized / merged-car detection
        // Boxes whose area exceeds MAX_CAR_BOX_AREA are treated as
        // containing multiple cars placed close together.  We split the
        // box into splitN equal slices along the longer axis and test each
        // slice anchor against the ROIs independently.
        // ----------------------------------------------------------------
        int32_t boxArea = (int32_t)w * h;
        bool oversized  = (boxArea > MAX_CAR_BOX_AREA);
        int  splitN     = 1;
        if (oversized) {
            splitN = (int)((float)boxArea / MAX_CAR_BOX_AREA + 0.5f);
            if (splitN < 2)          splitN = 2;
            if (splitN > MAX_CAR_SPLIT) splitN = MAX_CAR_SPLIT;
            dbg_printf("  ^ oversized (area=%ld px2) → splitting into %d virtual cars",
                       (long)boxArea, splitN);
        }

        // ----------------------------------------------------------------
        // ROI test: iterate through each virtual sub-box
        // ----------------------------------------------------------------
        uint8_t totalMask = 0;
        for (int s = 0; s < splitN; s++) {
            int sub_ax, sub_ay;
            if (splitN == 1) {
                // Normal single-car box — use the pre-computed display anchor
                sub_ax = disp_ax;
                sub_ay = disp_ay;
            } else {
                // Split along the longer dimension so each slice covers one car
                int sub_x, sub_y, sub_w, sub_h;
                if (w >= h) {                        // wider than tall → horizontal split
                    sub_w = w / splitN;  sub_h = h;
                    sub_x = x + s * sub_w;  sub_y = y;
                } else {                             // taller than wide → vertical split
                    sub_w = w;  sub_h = h / splitN;
                    sub_x = x;  sub_y = y + s * sub_h;
                }
                sub_ax = sub_x + sub_w / 2;
#if ROI_ANCHOR_BOTTOM_CENTER
                sub_ay = sub_y + sub_h;
#else
                sub_ay = sub_y + sub_h / 2;
#endif
                sub_ax = constrain(sub_ax, 0, EI_CAMERA_RAW_FRAME_BUFFER_COLS - 1);
                sub_ay = constrain(sub_ay, 0, EI_CAMERA_RAW_FRAME_BUFFER_ROWS - 1);
            }

            for (size_t r = 0; r < ROI_COUNT; r++) {
                if (point_in_poly(sub_ax, sub_ay, ROIS[r].pts, ROIS[r].n)) {
                    totalMask |= (1 << r);
                    g_roi_count[r]++;
                    frame_conf_sum[r] += bb.value;
                }
            }
        }

        // ----------------------------------------------------------------
        // Store for web display
        //   • Normal boxes  → always stored; green if inside ROI, grey if not.
        //   • Oversized boxes → only stored when g_show_oversized == true;
        //     always coloured orange regardless of ROI; mask forced to 0 so
        //     they are never misread as valid single-car detections.
        //     The label gets a ×N suffix to show the split count.
        // ----------------------------------------------------------------
        if ((!oversized || g_show_oversized) && g_box_count < MAX_BOXES) {
            Box &b = g_boxes[g_box_count++];
            if (oversized && splitN > 1)
                snprintf(b.label, sizeof(b.label), "%s*%d", bb.label, splitN);
            else
                strlcpy(b.label, bb.label, sizeof(b.label));
            b.value    = bb.value;
            b.x = x;  b.y = y;  b.w = w;  b.h = h;
            b.ax = disp_ax;  b.ay = disp_ay;
            b.mask     = oversized ? 0 : totalMask;  // oversized: never show as "inside"
            b.oversized = oversized;
            b.split_n   = (uint8_t)splitN;
        }
    }

    // A lane becomes reportable only after detections appear in three
    // consecutive inference frames and their mean confidence reaches 0.80.
    for (size_t r = 0; r < ROI_COUNT; r++) {
        if (g_roi_count[r] == 0) {
            g_traffic_streak[r] = 0;
            g_traffic_conf_sum[r] = 0.0f;
            g_traffic_count[r] = 0;
            memset(g_traffic_conf_history[r], 0, sizeof(g_traffic_conf_history[r]));
            continue;
        }
        if (g_traffic_streak[r] < TRAFFIC_REQUIRED_FRAMES) g_traffic_streak[r]++;
        for (int h = TRAFFIC_REQUIRED_FRAMES - 1; h > 0; h--) {
            g_traffic_conf_history[r][h] = g_traffic_conf_history[r][h - 1];
        }
        g_traffic_conf_history[r][0] = frame_conf_sum[r] / g_roi_count[r];
        g_traffic_conf_sum[r] = 0.0f;
        for (int h = 0; h < TRAFFIC_REQUIRED_FRAMES; h++) {
            g_traffic_conf_sum[r] += g_traffic_conf_history[r][h];
        }
        float average = g_traffic_conf_sum[r] / g_traffic_streak[r];
        g_traffic_count[r] = (g_traffic_streak[r] >= TRAFFIC_REQUIRED_FRAMES &&
                              average >= TRAFFIC_MIN_AVERAGE_CONFIDENCE)
                             ? g_roi_count[r] : 0;
    }

    dbg_printf("ROI counts:");
    for (size_t r = 0; r < ROI_COUNT; r++) {
        dbg_printf("  %s: %u", ROIS[r].name, (unsigned)g_roi_count[r]);
    }

    // Print the prediction results (classification)
#else
    dbg_printf("Predictions:");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        dbg_printf("  %s: %.5f", ei_classifier_inferencing_categories[i], result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    dbg_printf("Anomaly prediction: %.3f", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    dbg_printf("Visual anomalies:");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        dbg_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]",
                   bb.label,
                   bb.value,
                   (unsigned)bb.x,
                   (unsigned)bb.y,
                   (unsigned)bb.width,
                   (unsigned)bb.height);
    }
#endif

    // Publish: boxes + matching JPEG become visible to the web page together
    g_seq++;
    send_to_controller();          // push the debounced lane counts to the ESP32
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