/*
  ESP32-CAM Live Video Stream & Gemini AI Vision Analysis

  Features:
  1. Continuous MJPEG Video Stream accessible at http://<ESP32_IP>/stream
  2. Responsive Web Dashboard at http://<ESP32_IP>/
  3. "Analyze with Gemini" trigger button that captures the current camera
  frame, sends it to Google Gemini 2.0 Flash Vision API, and displays what
  Gemini sees.
  4. Non-blocking asynchronous background execution using FreeRTOS tasks to keep
     the web server and video stream running smoothly.

  Board: AI Thinker ESP32-CAM
  Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS) or Minimal SPIFFS
  PSRAM: Enabled (Required for Base64 image payload processing)
*/

#include "esp_camera.h"
#include "esp_http_server.h"
#include <NetworkClientSecure.h>
#include <WiFi.h>
typedef NetworkClientSecure WiFiClientSecure;
#include <HTTPClient.h>

// ============================================================================
// USER CONFIGURATION - REPLACE WITH YOUR CREDENTIALS
// ============================================================================
const char *WIFI_SSID = "Traffic_ESP";
const char *WIFI_PASSWORD = "Traffic_ESP";

// Gemini API Configuration
const char *GEMINI_API_KEY =
    "AQ.Ab8RN6K_lV7gslxwzIXq-ei3G-79iDt53qwGya79FRK4GiQ77Q";
const char *GEMINI_MODEL = "gemini-2.0-flash"; // Or gemini-1.5-flash
const char *PROMPT_TEXT =
    "List out everything you can see in this image in concise detail.";

// ============================================================================
// CAMERA PIN CONFIGURATION (AI-THINKER ESP32-CAM MODEL)
// ============================================================================
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_PIN 4

// ============================================================================
// GLOBAL STATE & VARIABLES
// ============================================================================
enum AnalysisState { IDLE, ANALYZING, COMPLETED, ERROR_STATE };
volatile AnalysisState currentStatus = IDLE;
String latestGeminiResult = "No analysis performed yet. Click 'Analyze with "
                            "Gemini' to analyze live frame.";
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// Semaphore to manage task triggering safely
SemaphoreHandle_t analyzeSemaphore = NULL;

// ============================================================================
// BASE64 ENCODER HELPER
// ============================================================================
static const char PROGMEM b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                           "abcdefghijklmnopqrstuvwxyz"
                                           "0123456789+/";

String base64Encode(const uint8_t *data, size_t length) {
  String encoded;
  encoded.reserve(((length + 2) / 3) * 4);

  size_t i = 0;
  while (i < length) {
    uint32_t octet_a = i < length ? data[i++] : 0;
    uint32_t octet_b = i < length ? data[i++] : 0;
    uint32_t octet_c = i < length ? data[i++] : 0;

    uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

    encoded += b64_alphabet[(triple >> 3 * 6) & 0x3F];
    encoded += b64_alphabet[(triple >> 2 * 6) & 0x3F];
    encoded += b64_alphabet[(triple >> 1 * 6) & 0x3F];
    encoded += b64_alphabet[(triple >> 0 * 6) & 0x3F];
  }

  size_t mod = length % 3;
  if (mod == 1) {
    encoded[encoded.length() - 1] = '=';
    encoded[encoded.length() - 2] = '=';
  } else if (mod == 2) {
    encoded[encoded.length() - 1] = '=';
  }

  return encoded;
}

// Helper function to escape special JSON characters in extracted Gemini
// response
String escapeJSON(String input) {
  String output = "";
  for (size_t i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (c == '"')
      output += "\\\"";
    else if (c == '\\')
      output += "\\\\";
    else if (c == '\b')
      output += "\\b";
    else if (c == '\f')
      output += "\\f";
    else if (c == '\n')
      output += "\\n";
    else if (c == '\r')
      output += "\\r";
    else if (c == '\t')
      output += "\\t";
    else
      output += c;
  }
  return output;
}

// ============================================================================
// GEMINI VISION API TASK (BACKGROUND FREERTOS TASK)
// ============================================================================
void geminiTask(void *pvParameters) {
  for (;;) {
    // Wait until a button click triggers the semaphore
    if (xSemaphoreTake(analyzeSemaphore, portMAX_DELAY) == pdTRUE) {
      currentStatus = ANALYZING;
      Serial.println("[Gemini Task] Starting frame capture...");

      // Briefly turn on flash LED (optional visual feedback, uncomment if
      // desired) digitalWrite(FLASH_LED_PIN, HIGH); delay(100);
      // digitalWrite(FLASH_LED_PIN, LOW);

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("[Gemini Task] Camera capture failed!");
        latestGeminiResult = "Error: Failed to capture camera frame.";
        currentStatus = ERROR_STATE;
        continue;
      }

      Serial.printf("[Gemini Task] Captured frame: %d bytes (format %d)\n",
                    fb->len, fb->format);

      // Base64 encode the captured JPEG frame
      Serial.println("[Gemini Task] Base64 encoding image...");
      String base64Image = base64Encode(fb->buf, fb->len);

      // Return frame buffer back to driver as early as possible
      esp_camera_fb_return(fb);

      Serial.printf("[Gemini Task] Base64 string length: %d\n",
                    base64Image.length());
      Serial.println("[Gemini Task] Connecting to Gemini API...");

      WiFiClientSecure client;
      client.setInsecure(); // Skip certificate verification for speed & memory
                            // efficiency

      HTTPClient http;
      String url = "https://generativelanguage.googleapis.com/v1beta/models/";
      url += GEMINI_MODEL;
      url += ":generateContent?key=";
      url += GEMINI_API_KEY;

      if (http.begin(client, url)) {
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(15000); // 15s timeout for Gemini processing

        // Construct JSON Payload for Gemini Vision API
        String jsonPayload = "{\"contents\":[{\"parts\":[";
        jsonPayload += "{\"text\":\"";
        jsonPayload += PROMPT_TEXT;
        jsonPayload += "\"},";
        jsonPayload +=
            "{\"inline_data\":{\"mime_type\":\"image/jpeg\",\"data\":\"";
        jsonPayload += base64Image;
        jsonPayload += "\"}}";
        jsonPayload += "]}]}";

        // Clear heavy base64 string from memory immediately
        base64Image = "";

        Serial.println("[Gemini Task] Sending HTTPS POST request...");
        int httpCode = http.POST(jsonPayload);
        jsonPayload = ""; // Free RAM

        if (httpCode > 0) {
          Serial.printf("[Gemini Task] HTTP Response Code: %d\n", httpCode);
          String response = http.getString();

          if (httpCode == HTTP_CODE_OK) {
            // Extract text response from Gemini's JSON payload
            // Looks for "text": "..." inside candidates
            int textKeyIndex = response.indexOf("\"text\": \"");
            if (textKeyIndex != -1) {
              int start = textKeyIndex + 9;
              int end = response.indexOf("\"", start);

              // Handle potential escaped quotes or multiline strings
              String extractedText = "";
              while (end != -1 && response.charAt(end - 1) == '\\') {
                end = response.indexOf("\"", end + 1);
              }
              if (end != -1) {
                extractedText = response.substring(start, end);
                // Unescape standard newlines and quotes
                extractedText.replace("\\n", "\n");
                extractedText.replace("\\\"", "\"");
                latestGeminiResult = extractedText;
              } else {
                latestGeminiResult = response;
              }
            } else {
              latestGeminiResult = response;
            }
            currentStatus = COMPLETED;
            Serial.println("[Gemini Task] Success! Analysis updated.");
          } else {
            Serial.printf("[Gemini Task] API Error (%d): %s\n", httpCode,
                          response.c_str());
            latestGeminiResult =
                "API Error (" + String(httpCode) + "): " + response;
            currentStatus = ERROR_STATE;
          }
        } else {
          Serial.printf("[Gemini Task] HTTP POST failed: %s\n",
                        http.errorToString(httpCode).c_str());
          latestGeminiResult = "Network Error: " + http.errorToString(httpCode);
          currentStatus = ERROR_STATE;
        }

        http.end();
      } else {
        Serial.println("[Gemini Task] Unable to connect to endpoint.");
        latestGeminiResult =
            "Connection Error: Failed to initiate HTTPS connection.";
        currentStatus = ERROR_STATE;
      }
    }
  }
}

// ============================================================================
// WEB INTERFACE (HTML & JS EMBEDDED IN HEADER)
// ============================================================================
#include "index_html.h"

// ============================================================================
// HTTP HANDLERS
// ============================================================================

// 1. Dashboard Handler (/)
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// 2. MJPEG Stream Handler (/stream)
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char ts_buf[64];

  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Stream] Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(
        ts_buf, sizeof(ts_buf),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        fb->len);
    res = httpd_resp_send_chunk(req, ts_buf, hlen);
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, "\r\n", 2);
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      break;
    }
  }

  return res;
}

// 3. Analyze Request Handler (/analyze)
static esp_err_t analyze_handler(httpd_req_t *req) {
  if (currentStatus != ANALYZING) {
    // Trigger background task using semaphore
    xSemaphoreGive(analyzeSemaphore);
  }

  httpd_resp_set_type(req, "application/json");
  const char *jsonResp = "{\"status\":\"queued\"}";
  return httpd_resp_send(req, jsonResp, strlen(jsonResp));
}

// 4. Status/Result Handler (/result)
static esp_err_t result_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");

  String statusStr = "idle";
  if (currentStatus == ANALYZING)
    statusStr = "analyzing";
  else if (currentStatus == COMPLETED)
    statusStr = "completed";
  else if (currentStatus == ERROR_STATE)
    statusStr = "error";

  String jsonStr = "{\"status\":\"" + statusStr + "\",\"result\":\"" +
                   escapeJSON(latestGeminiResult) + "\"}";
  return httpd_resp_send(req, jsonStr.c_str(), jsonStr.length());
}

// ============================================================================
// WEB SERVER INITIALIZATION
// ============================================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = index_handler,
                           .user_ctx = NULL};

  httpd_uri_t stream_uri = {.uri = "/stream",
                            .method = HTTP_GET,
                            .handler = stream_handler,
                            .user_ctx = NULL};

  httpd_uri_t analyze_uri = {.uri = "/analyze",
                             .method = HTTP_POST,
                             .handler = analyze_handler,
                             .user_ctx = NULL};

  httpd_uri_t result_uri = {.uri = "/result",
                            .method = HTTP_GET,
                            .handler = result_handler,
                            .user_ctx = NULL};

  Serial.println("[Server] Starting web server on port 80...");
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &analyze_uri);
    httpd_register_uri_handler(camera_httpd, &result_uri);
  }

  // Create dedicated HTTP server instance for video stream to prevent socket
  // blocking
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ============================================================================
// INITIAL SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n--- ESP32-CAM Gemini Vision Assistant ---");

  if (psramFound()) {
    Serial.println("PSRAM is initialized and working!");
    Serial.print("Free PSRAM: ");
    Serial.print(ESP.getFreePsram());
    Serial.println(" bytes");
  } else {
    Serial.println("ERROR: PSRAM not found!");
  }

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Reset camera power hardware before init
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH); // Power down camera
  delay(20);
  digitalWrite(PWDN_GPIO_NUM, LOW);  // Power up camera
  delay(20);

  // Initialize Camera
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size & quality configuration
  if (psramFound()) {
    Serial.println("[Setup] PSRAM Detected! Using VGA resolution.");
    config.frame_size = FRAMESIZE_VGA; // 640x480
    config.jpeg_quality = 12;          // 10-63 (lower = higher quality)
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println(
        "[Setup] Warning: PSRAM NOT Found! Reducing resolution to QVGA.");
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 14;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Setup] Camera init failed with error 0x%x\n", err);
    return;
  }

  // Connect to Wi-Fi
  Serial.printf("[Setup] Connecting to Wi-Fi SSID: %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Setup] Wi-Fi Connected!");
  Serial.print("[Setup] Local IP Address: http://");
  Serial.println(WiFi.localIP());

  // Create Semaphore for triggering background task
  analyzeSemaphore = xSemaphoreCreateBinary();

  // Create Gemini API processing task pinned to Core 1
  xTaskCreatePinnedToCore(geminiTask,   /* Task function */
                          "GeminiTask", /* Task name */
                          10240,        /* Stack size in words (10KB) */
                          NULL,         /* Task parameter */
                          1,            /* Task priority */
                          NULL,         /* Task handle */
                          1             /* Core ID (Core 1) */
  );

  // Start HTTP Web Servers
  startCameraServer();

  Serial.println(
      "[Setup] Ready! Open browser and navigate to the IP address above.");
}

void loop() {
  // FreeRTOS and HTTP server handles work in background tasks.
  delay(10000);
}
