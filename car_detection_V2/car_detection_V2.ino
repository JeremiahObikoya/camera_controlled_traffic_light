/* Edge Impulse Arduino examples
   Copyright (c) 2022 EdgeImpulse Inc.

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <car_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"

#include <WiFi.h>
#include <WebServer.h>

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

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

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture
static uint8_t *raw_image = nullptr;
static uint8_t *model_image = nullptr;
static size_t raw_image_size = 0;
static size_t model_image_size = 0;

struct DetectionBox {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
    const char *label;
};

static DetectionBox detections[32];
static uint32_t detection_count = 0;
static DetectionBox displayed_detections[32];
static uint32_t displayed_detection_count = 0;
static uint8_t *pending_frame_jpeg = nullptr;
static size_t pending_frame_jpeg_len = 0;
static uint8_t *latest_frame_jpeg = nullptr;
static size_t latest_frame_jpeg_len = 0;

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

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12, //0-63 lower number means higher quality
  // Two PSRAM frame buffers prevent the camera task from stalling while the
  // JPEG frame is being converted for inference.
  .fb_count = 2,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_LATEST,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void publishInferenceResult(const ei_impulse_result_t &result);

/* Web-based serial monitor (SoftAP) ---------------------------------------- */
const char* AP_SSID     = "ESP32-CAM-Monitor";
const char* AP_PASSWORD = "12345678";   // 8+ chars for WPA2, or use WiFi.softAP(AP_SSID) for open AP

WebServer server(80);

#define LOG_BUF_SIZE 8192
static char logBuffer[LOG_BUF_SIZE];
static size_t logHead = 0;
static bool logWrapped = false;

void webLogWrite(const char *msg, size_t len) {
  for (size_t i = 0; i < len; i++) {
    logBuffer[logHead] = msg[i];
    logHead = (logHead + 1) % LOG_BUF_SIZE;
    if (logHead == 0) logWrapped = true;
  }
}

String getLogText() {
  String out;
  if (!logWrapped) {
    out.reserve(logHead);
    for (size_t i = 0; i < logHead; i++) out += logBuffer[i];
  } else {
    out.reserve(LOG_BUF_SIZE);
    for (size_t i = 0; i < LOG_BUF_SIZE; i++)
      out += logBuffer[(logHead + i) % LOG_BUF_SIZE];
  }
  return out;
}

void weblog_vprintf(const char *fmt, va_list args) {
  char buf[256];
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  if (len < 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
  Serial.write((const uint8_t*)buf, len);   // USB serial still works too
  webLogWrite(buf, len);
}

void weblog_printf(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  weblog_vprintf(fmt, args);
  va_end(args);
}

// Overrides the weak ei_printf() in the Edge Impulse SDK, so every existing
// ei_printf(...) call already in your loop()/setup() is mirrored to the web
// log with zero changes needed there.
void ei_printf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  weblog_vprintf(format, args);
  va_end(args);
}

// ==========================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>ESP32-CAM Monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{background:#111;color:#0f0;font-family:monospace;margin:0;padding:10px;}
  pre{white-space:pre-wrap;word-wrap:break-word;}
  h3{color:#eee;font-family:sans-serif;}
        .capture-panel{position:fixed;right:16px;bottom:16px;z-index:10;width:min(320px,calc(100vw - 32px));padding:8px;background:rgba(17,17,17,.95);border:1px solid #555;box-shadow:0 2px 8px #000;}
        #captureButton{width:100%;padding:10px 14px;background:#16803c;color:#fff;border:0;border-radius:4px;cursor:pointer;font-weight:bold;}
        #captureButton:disabled{opacity:.6;cursor:wait;}
    #snapshot{display:block;width:100%;height:auto;max-height:45vh;object-fit:contain;background:#000;border:1px solid #555;}
    #captureStatus{margin:6px 0;font-size:12px;white-space:normal;}
</style></head>
<body>
<h3>ESP32-CAM Serial Monitor</h3>
<div class="capture-panel">
    <canvas id="snapshot" aria-label="Captured image with detections"></canvas>
    <p id="captureStatus">No snapshot captured.</p>
    <button id="captureButton" onclick="captureSnapshot()">Show Latest Inference</button>
</div>
<pre id="log">Connecting...</pre>
<script>
  const el = document.getElementById('log');
    const captureButton = document.getElementById('captureButton');
    const captureStatus = document.getElementById('captureStatus');
    async function captureSnapshot() {
        captureButton.disabled = true;
        captureStatus.textContent = 'Loading the most recent analyzed frame...';
        try {
            const response = await fetch('/capture');
            if (!response.ok) throw new Error(await response.text());
            const imageBlob = await response.blob();
            const imageUrl = URL.createObjectURL(imageBlob);
            const image = new Image();
            await new Promise((resolve, reject) => {
                image.onload = resolve;
                image.onerror = () => reject(new Error('Image could not be decoded'));
                image.src = imageUrl;
            });
            try {
                const canvas = document.getElementById('snapshot');
                canvas.width = image.naturalWidth;
                canvas.height = image.naturalHeight;
                const context = canvas.getContext('2d');
                context.drawImage(image, 0, 0);
                const boxesResponse = await fetch('/detections');
                if (!boxesResponse.ok) throw new Error('Could not load detections');
                const boxes = await boxesResponse.json();
                context.strokeStyle = '#00ff00';
                context.lineWidth = Math.max(2, canvas.width / 160);
                context.font = 'bold 14px sans-serif';
                boxes.forEach(box => {
                    context.strokeRect(box.x, box.y, box.width, box.height);
                    context.fillStyle = '#00ff00';
                    context.fillText(box.label, box.x + 3, Math.max(14, box.y - 4));
                });
                captureStatus.textContent = boxes.length
                    ? 'Showing the latest analyzed frame with its detections.'
                    : 'Showing the latest analyzed frame. No objects detected.';
            } finally {
                URL.revokeObjectURL(imageUrl);
            }
        } catch (error) {
            captureStatus.textContent = error.message;
        } finally {
            captureButton.disabled = false;
        }
    }
  async function poll() {
    try {
      const r = await fetch('/log');
      el.textContent = await r.text();
      window.scrollTo(0, document.body.scrollHeight);
    } catch (e) {}
    setTimeout(poll, 700);
  }
  poll();
</script>
</body></html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }
void handleLog()  { server.send(200, "text/plain", getLogText()); }

void handleCapture() {
    if (latest_frame_jpeg == nullptr || latest_frame_jpeg_len == 0) {
        server.send(503, "text/plain", "Waiting for the first successful inference");
        return;
    }

    displayed_detection_count = detection_count;
    memcpy(displayed_detections, detections, detection_count * sizeof(DetectionBox));

    server.setContentLength(latest_frame_jpeg_len);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "image/jpeg", "");
    server.sendContent((const char *)latest_frame_jpeg, latest_frame_jpeg_len);
}

void handleDetections() {
    String json = "[";
    for (uint32_t i = 0; i < displayed_detection_count; i++) {
        if (i > 0) json += ",";
        json += "{\"x\":" + String(displayed_detections[i].x);
        json += ",\"y\":" + String(displayed_detections[i].y);
        json += ",\"width\":" + String(displayed_detections[i].width);
        json += ",\"height\":" + String(displayed_detections[i].height);
        json += ",\"label\":\"" + String(displayed_detections[i].label) + "\"}";
    }
    json += "]";
    server.send(200, "application/json", json);
}

void publishInferenceResult(const ei_impulse_result_t &result) {
    detection_count = 0;
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    int crop_width;
    int crop_height;
    ei::image::processing::calculate_crop_dims(
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT,
            crop_width,
            crop_height);
    const uint32_t crop_x = (EI_CAMERA_RAW_FRAME_BUFFER_COLS - crop_width) / 2;
    const uint32_t crop_y = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS - crop_height) / 2;

    for (uint32_t i = 0; i < result.bounding_boxes_count && detection_count < 32; i++) {
        const ei_impulse_result_bounding_box_t &box = result.bounding_boxes[i];
        if (box.value == 0) continue;

        detections[detection_count].x = crop_x + box.x * crop_width / EI_CLASSIFIER_INPUT_WIDTH;
        detections[detection_count].y = crop_y + box.y * crop_height / EI_CLASSIFIER_INPUT_HEIGHT;
        detections[detection_count].width = box.width * crop_width / EI_CLASSIFIER_INPUT_WIDTH;
        detections[detection_count].height = box.height * crop_height / EI_CLASSIFIER_INPUT_HEIGHT;
        detections[detection_count].label = box.label;
        detection_count++;
    }
#endif

    free(latest_frame_jpeg);
    latest_frame_jpeg = pending_frame_jpeg;
    latest_frame_jpeg_len = pending_frame_jpeg_len;
    pending_frame_jpeg = nullptr;
    pending_frame_jpeg_len = 0;
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/log", handleLog);
        server.on("/capture", HTTP_GET, handleCapture);
        server.on("/detections", HTTP_GET, handleDetections);
    server.begin();
}



/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
while (!Serial);

WiFi.mode(WIFI_AP);
WiFi.softAP(AP_SSID, AP_PASSWORD);
weblog_printf("SoftAP started: \"%s\" -> http://%s/\n",
              AP_SSID, WiFi.softAPIP().toString().c_str());
setupWebServer();

weblog_printf("Edge Impulse Inferencing Demo\n");   // was Serial.println(...)
if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
}
else {
    ei_printf("Camera initialized\r\n");

    const size_t raw_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE;
    const size_t model_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * EI_CAMERA_FRAME_BYTE_SIZE;
    raw_image_size = raw_size;
    model_image_size = model_size;
    raw_image = (uint8_t*)heap_caps_malloc(raw_image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // Keep the classifier input in internal 8-bit RAM. Some ESP32 DSP/model
    // builds cannot safely write their tensor input through a PSRAM pointer.
    model_image = raw_image;
    model_image_size = raw_image_size;
    if (raw_image == nullptr || model_image == nullptr) {
        ei_printf("ERR: Failed to allocate image buffers in PSRAM (raw=%u, model=%u, free=%u)!\n",
                  (unsigned)raw_image_size, (unsigned)model_image_size,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        heap_caps_free(raw_image);
        if (model_image != raw_image) heap_caps_free(model_image);
        raw_image = nullptr;
        model_image = nullptr;
        raw_image_size = 0;
        model_image_size = 0;
    }
} 

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    server.handleClient();

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    if (raw_image == nullptr || model_image == nullptr) {
        return;
    }

    snapshot_buf = raw_image;

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, model_image) == false) {
        ei_printf("Failed to capture image\r\n");
        heap_caps_free(pending_frame_jpeg);
        pending_frame_jpeg = nullptr;
        pending_frame_jpeg_len = 0;
        snapshot_buf = nullptr;
        return;
    }

    snapshot_buf = raw_image;

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        heap_caps_free(pending_frame_jpeg);
        pending_frame_jpeg = nullptr;
        pending_frame_jpeg_len = 0;
        snapshot_buf = nullptr;
        return;
    }

    publishInferenceResult(result);

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    snapshot_buf = nullptr;

}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
if (err != ESP_OK) {
  weblog_printf("Camera init failed with error 0x%x\n", err);   // was Serial.printf(...)
  return false;
}

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
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

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = nullptr;
    for (uint8_t attempt = 0; attempt < 3 && fb == nullptr; attempt++) {
        fb = esp_camera_fb_get();
        if (fb == nullptr) delay(20);
    }

    if (!fb) {
        ei_printf("Camera capture failed (free heap=%u, free PSRAM=%u)\n",
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return false;
    }

    if (fb->width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || fb->height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS || fb->format != PIXFORMAT_JPEG) {
        ei_printf("ERR: Unexpected camera frame: %ux%u format %d (expected 320x240 JPEG)\n",
                  fb->width, fb->height, fb->format);
        esp_camera_fb_return(fb);
        return false;
    }

    const size_t raw_required = (size_t)EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                                EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                                EI_CAMERA_FRAME_BYTE_SIZE;
    const size_t output_required = (size_t)img_width * img_height *
                                   EI_CAMERA_FRAME_BYTE_SIZE;
    if (snapshot_buf == nullptr || out_buf == nullptr ||
        raw_image_size < raw_required || model_image_size < output_required) {
        ei_printf("ERR: Image buffer is not available\n");
        esp_camera_fb_return(fb);
        return false;
    }

    uint8_t *frame_copy = (uint8_t *)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (frame_copy == nullptr) {
        esp_camera_fb_return(fb);
        ei_printf("ERR: Failed to allocate JPEG frame copy\n");
        return false;
    }
    memcpy(frame_copy, fb->buf, fb->len);
    size_t frame_copy_len = fb->len;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
        heap_caps_free(frame_copy);
       return false;
   }

    heap_caps_free(pending_frame_jpeg);
    pending_frame_jpeg = frame_copy;
    pending_frame_jpeg_len = frame_copy_len;

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    } else {
        memcpy(out_buf, snapshot_buf, img_width * img_height * EI_CAMERA_FRAME_BYTE_SIZE);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
