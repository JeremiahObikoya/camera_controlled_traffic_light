#include <MFRC522.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------------------------------------------------------------------
// Link to the ESP32-CAM (it runs the SoftAP "CarDebug")
// UDP_PORT must match the camera sketch.
// ---------------------------------------------------------------------------
const char *CAMERA_SSID = "CarDebug";
const char *CAMERA_PASSWORD = "12345678";
const IPAddress CAMERA_IP(192, 168, 4, 1);
const uint16_t UDP_PORT = 4210;

const unsigned long HEARTBEAT_INTERVAL_MS = 500;  // controller -> camera "I'm alive"
const unsigned long LINK_TIMEOUT_MS = 8000;       // no camera packet for this long = link lost
                                                   // NOTE: must be > the ESP32-CAM inference cycle time
                                                   // (DSP + classification can take 2-5 s per frame)

// ---------------------------------------------------------------------------
// Traffic policy
// ---------------------------------------------------------------------------
const unsigned long LANE_SWITCH_INTERVAL = 30000; // TIME mode: seconds per lane
const unsigned long MIN_GREEN_MS = 10000;         // CAMERA mode: a lane keeps green at least this long
const unsigned long MAX_GREEN_MS = 60000;         // CAMERA mode: after this, give a waiting lane a turn (0 = off)

// --- Shared SPI Pins ---
#define SPI_SCK 18
#define SPI_MOSI 23
#define SPI_MISO 19

// --- SHARED RFID Reset ---
// Wire the RST pin of ALL THREE readers to pin 22
#define SHARED_RFID_RST 22

// --- RFID Readers per Lane ---
// Lane 1
#define RFID1_SS 4
MFRC522 mfrc522_1(RFID1_SS, SHARED_RFID_RST);

// Lane 2 (SS on pin 21)
#define RFID2_SS 21
MFRC522 mfrc522_2(RFID2_SS, SHARED_RFID_RST);

// Lane 3
#define RFID3_SS 2
MFRC522 mfrc522_3(RFID3_SS, SHARED_RFID_RST);

// Array of pointers for easy iteration
MFRC522 *rfidReaders[3] = {&mfrc522_1, &mfrc522_2, &mfrc522_3};
const int laneForReader[3] = {1, 2, 3}; // lane number for each reader

// --- Traffic Light Pins (unchanged) ---
#define LANE2_RED 32
#define LANE2_YELLOW 33
#define LANE2_GREEN 25
#define LANE3_RED 26
#define LANE3_YELLOW 27
#define LANE3_GREEN 14
#define LANE1_RED 12
#define LANE1_YELLOW 15
#define LANE1_GREEN 13

const int totalLanes = 3;
volatile int activeLane = 1; // currently green lane (read by the comms task)

unsigned long lastLaneSwitchTime = 0;

// List of authorized UIDs (only these 3 tags are recognized)
byte authorizedUIDs[][4] = {
    {0xDE, 0xD2, 0x47, 0x06}, // Tag 1
    {0xD1, 0x60, 0x3E, 0x06}, // Tag 2
    {0x87, 0x5D, 0x42, 0x06}  // Tag 3
};
int numAuthorizedTags = 3; // How many tags are registered

// ---------------------------------------------------------------------------
// State received from the camera. Written by the comms task, read by loop().
// ---------------------------------------------------------------------------
struct CamState {
  bool everRx;          // have we ever received a camera packet
  bool modeKnown;       // have we received a mode yet
  bool cameraMode;      // true = CAMERA control, false = TIME control (set on the camera web page)
  uint8_t counts[3];    // debounced vehicle count for lane 1..3
  uint32_t seq;
  uint32_t lastRxMs;
};
static CamState g_cam = {false, false, true, {0, 0, 0}, 0, 0};
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_effectiveCamera = false; // what the controller is really doing (sent in heartbeat)

// ---------------------------------------------------------------------------
// Thread-safe log queue: commTask (Core 0) enqueues, loop() (Core 1) drains.
// Lock-free single-producer / single-consumer ring buffer — no mutex needed.
// ---------------------------------------------------------------------------
#define LOG_Q_SLOTS  10
#define LOG_Q_MSGLEN 96
static char     g_logQ[LOG_Q_SLOTS][LOG_Q_MSGLEN];
static volatile uint8_t g_logQW = 0;  // write index, updated only by commTask
static volatile uint8_t g_logQR = 0;  // read  index, updated only by loop()

static void qLog(const char *msg) {
  uint8_t next = (g_logQW + 1) % LOG_Q_SLOTS;
  if (next == g_logQR) return;           // queue full — drop silently
  strlcpy(g_logQ[g_logQW], msg, LOG_Q_MSGLEN);
  g_logQW = next;                        // publish (atomic byte write on ESP32)
}

static void qLogf(const char *fmt, ...) {
  char buf[LOG_Q_MSGLEN];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  qLog(buf);
}

static void drainLog() {               // call ONLY from loop() / Core 1
  while (g_logQR != g_logQW) {
    Serial.println(g_logQ[g_logQR]);
    g_logQR = (g_logQR + 1) % LOG_Q_SLOTS;
  }
}

// --- Function Prototypes ---
void setLights(int lane, bool red, bool yellow, bool green);
void switchToLane(int newActiveLane);
void performRedToGreenTransition(int targetLane);
void performGreenToRedTransition(int currentLane);
bool isTagRegistered(MFRC522 &reader);
void printUIDToSerial(MFRC522 &reader);
void runTrafficLogic();
void handleRfid();

// ===========================================================================
// COMMS TASK  (runs on its own core/task, so the blocking light delays in
// loop() can never stall the Wi-Fi link or the heartbeat)
// ===========================================================================
static WiFiUDP udp;
static bool udpUp = false;
static bool wifiWasUp = false;
static uint32_t lastWifiTry = 0;
static uint32_t lastHeartbeat = 0;

static void parseCamPacket(const char *buf) {
  // Format: CAM,<seq>,<C|T>,<lane1>,<lane2>,<lane3>
  unsigned seq = 0;
  char m = 'C';
  int a = 0, b = 0, c = 0;
  if (sscanf(buf, "CAM,%u,%c,%d,%d,%d", &seq, &m, &a, &b, &c) != 5) return;

  bool newMode = (m == 'C');
  bool first, changed;
  portENTER_CRITICAL(&g_mux);
  first = !g_cam.modeKnown;
  changed = g_cam.modeKnown && (g_cam.cameraMode != newMode);
  g_cam.cameraMode = newMode;
  g_cam.modeKnown = true;
  g_cam.counts[0] = (uint8_t)constrain(a, 0, 255);
  g_cam.counts[1] = (uint8_t)constrain(b, 0, 255);
  g_cam.counts[2] = (uint8_t)constrain(c, 0, 255);
  g_cam.seq = seq;
  g_cam.lastRxMs = millis();
  g_cam.everRx = true;
  portEXIT_CRITICAL(&g_mux);

  if (changed) {
    qLogf("*** MODE CHANGED: %s -> %s (set on camera web page) ***",
          newMode ? "TIME" : "CAMERA", newMode ? "CAMERA" : "TIME");
  } else if (first) {
    qLogf("Camera reports control mode: %s", newMode ? "CAMERA" : "TIME");
  }
}

static void commTask(void *) {
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiWasUp) {
        wifiWasUp = true;
        qLogf("Wi-Fi: connected to '%s', my IP %s", CAMERA_SSID,
              WiFi.localIP().toString().c_str());
      }
      if (!udpUp) {
        udp.begin(UDP_PORT);
        udpUp = true;
      }

      // receive: drain everything that arrived
      int guard = 0;
      while (udp.parsePacket() > 0 && guard++ < 16) {
        char buf[64];
        int n = udp.read(buf, sizeof(buf) - 1);
        if (n > 0) {
          buf[n] = 0;
          parseCamPacket(buf);
        }
      }

      // send heartbeat: "HB,<green lane>,<C|T>"
      if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        char msg[24];
        snprintf(msg, sizeof(msg), "HB,%d,%c", (int)activeLane, g_effectiveCamera ? 'C' : 'T');
        udp.beginPacket(CAMERA_IP, UDP_PORT);
        udp.print(msg);
        udp.endPacket();
      }
    } else {
      if (wifiWasUp) {
        wifiWasUp = false;
        qLog("Wi-Fi: lost connection to camera AP");
      }
      if (udpUp) {
        udp.stop();
        udpUp = false;
      }
      if (now - lastWifiTry >= 5000) {
        lastWifiTry = now;
        qLog("Wi-Fi: (re)connecting to camera AP...");
        WiFi.disconnect();
        WiFi.begin(CAMERA_SSID, CAMERA_PASSWORD);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // Wi-Fi: start connecting, but don't block the lights waiting for it
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(CAMERA_SSID, CAMERA_PASSWORD);
  lastWifiTry = millis();
  xTaskCreatePinnedToCore(commTask, "comm", 6144, nullptr, 1, nullptr, 0);

  // Init LED pins
  pinMode(LANE1_RED, OUTPUT);
  pinMode(LANE1_YELLOW, OUTPUT);
  pinMode(LANE1_GREEN, OUTPUT);
  pinMode(LANE2_RED, OUTPUT);
  pinMode(LANE2_YELLOW, OUTPUT);
  pinMode(LANE2_GREEN, OUTPUT);
  pinMode(LANE3_RED, OUTPUT);
  pinMode(LANE3_YELLOW, OUTPUT);
  pinMode(LANE3_GREEN, OUTPUT);

  // SPI bus (shared)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1); // -1 for default SS not used here

  // Initialize all three RFID readers
  for (int i = 0; i < 3; i++) {
    rfidReaders[i]->PCD_Init();
    delay(10);
  }
  Serial.println("3 RFID Readers initialized successfully.");

  // Start with all red
  setLights(1, true, false, false);
  setLights(2, true, false, false);
  setLights(3, true, false, false);

  // Set initial state to Lane 1 green
  activeLane = 1;
  performRedToGreenTransition(1);
  lastLaneSwitchTime = millis();
}

// --- Main Loop ---
void loop() {
  drainLog();           // flush any messages queued by commTask (thread-safe)
  runTrafficLogic();
  handleRfid();
  delay(50);
}

// ===========================================================================
// TRAFFIC LOGIC  (camera mode / time mode / automatic fallback)
// ===========================================================================
void runTrafficLogic() {
  static bool prevLinkUp = false;
  static bool prevUseCamera = false;
  static uint8_t prevCounts[3] = {255, 255, 255};

  CamState cs;
  portENTER_CRITICAL(&g_mux);
  cs = g_cam;
  portEXIT_CRITICAL(&g_mux);

  // (millis() is read AFTER the snapshot so lastRxMs can never be "in the future")
  bool linkUp = cs.everRx && (millis() - cs.lastRxMs) < LINK_TIMEOUT_MS;
  if (linkUp != prevLinkUp) {
    prevLinkUp = linkUp;
    Serial.println(linkUp ? "Camera link: CONNECTED" : "Camera link: LOST");
  }

  // Camera control is only used when the camera says CAMERA *and* the link is alive.
  // If the link dies in camera mode we fall back to the timer instead of freezing.
  bool useCamera = cs.cameraMode && linkUp;
  g_effectiveCamera = useCamera;
  if (useCamera != prevUseCamera) {
    prevUseCamera = useCamera;
    if (useCamera) {
      Serial.println("Traffic control: CAMERA (lane with most cars gets green)");
    } else {
      if (cs.cameraMode) Serial.println("Traffic control: TIME (camera link lost - automatic fallback)");
      else               Serial.println("Traffic control: TIME (selected on camera web page)");
      lastLaneSwitchTime = millis(); // give the current lane a full timer interval
    }
  }

  if (linkUp && memcmp(prevCounts, cs.counts, 3) != 0) {
    memcpy(prevCounts, cs.counts, 3);
    Serial.printf("Camera traffic: ROI1=%u ROI2=%u ROI3=%u\n", cs.counts[0], cs.counts[1], cs.counts[2]);
  }

  unsigned long greenFor = millis() - lastLaneSwitchTime;

  if (useCamera) {
    if (greenFor < MIN_GREEN_MS) return; // avoid flip-flopping

    // Busiest lane wins; ties keep the current lane.
    int best = activeLane;
    for (int lane = 1; lane <= totalLanes; lane++) {
      if (cs.counts[lane - 1] > cs.counts[best - 1]) best = lane;
    }
    if (best != activeLane) {
      Serial.printf("Camera selected Lane %d (%u cars) over Lane %d (%u cars)\n",
                    best, cs.counts[best - 1], (int)activeLane, cs.counts[activeLane - 1]);
      switchToLane(best);
      return;
    }

    // Fairness: don't let one lane hold green forever while others wait
    if (MAX_GREEN_MS > 0 && greenFor >= MAX_GREEN_MS) {
      int other = 0;
      for (int lane = 1; lane <= totalLanes; lane++) {
        if (lane == activeLane || cs.counts[lane - 1] == 0) continue;
        if (other == 0 || cs.counts[lane - 1] > cs.counts[other - 1]) other = lane;
      }
      if (other != 0) {
        Serial.printf("Lane %d held green for %lus - giving Lane %d (%u cars) a turn\n",
                      (int)activeLane, greenFor / 1000UL, other, cs.counts[other - 1]);
        switchToLane(other);
      }
    }
  } else if (greenFor >= LANE_SWITCH_INTERVAL) {
    int nextLane = (activeLane % totalLanes) + 1;
    Serial.print("\nTimer elapsed. Switching from Lane ");
    Serial.print((int)activeLane);
    Serial.print(" to Lane ");
    Serial.println(nextLane);
    switchToLane(nextLane);
  }
}

// ===========================================================================
// RFID priority (unchanged behaviour)
// ===========================================================================
void handleRfid() {
  for (int i = 0; i < 3; i++) {
    MFRC522 *reader = rfidReaders[i];
    if (reader->PICC_IsNewCardPresent() && reader->PICC_ReadCardSerial()) {
      Serial.print("\nRFID Tag detected on Lane ");
      Serial.print(laneForReader[i]);
      Serial.print(" | UID: ");
      printUIDToSerial(*reader);

      if (isTagRegistered(*reader)) {
        int detectedLane = laneForReader[i];
        Serial.print("SUCCESS: Registered Tag! Granting priority to Lane ");
        Serial.println(detectedLane);

        if (detectedLane != activeLane) {
          switchToLane(detectedLane);
        } else {
          Serial.print("Lane ");
          Serial.print(detectedLane);
          Serial.println(" already has Green. Priority maintained.");
          lastLaneSwitchTime = millis(); // Reset timer for active lane
        }
      } else {
        Serial.println("DENIED: Unregistered tag ignored.");
      }
      reader->PICC_HaltA();
      reader->PCD_StopCrypto1();
      delay(200); // avoid rapid re-triggering for same tag
    }
  }
}

// --- Traffic Light Functions ---
void setLights(int lane, bool red, bool yellow, bool green) {
  int r, y, g;
  switch (lane) {
  case 1:
    r = LANE1_RED;
    y = LANE1_YELLOW;
    g = LANE1_GREEN;
    break;
  case 2:
    r = LANE2_RED;
    y = LANE2_YELLOW;
    g = LANE2_GREEN;
    break;
  case 3:
    r = LANE3_RED;
    y = LANE3_YELLOW;
    g = LANE3_GREEN;
    break;
  default:
    return;
  }
  digitalWrite(r, red ? HIGH : LOW);
  digitalWrite(y, yellow ? HIGH : LOW);
  digitalWrite(g, green ? HIGH : LOW);
}

void performRedToGreenTransition(int targetLane) {
  Serial.print("Lane ");
  Serial.print(targetLane);
  Serial.println(": Red -> Green");
  setLights(targetLane, true, false, false);
  delay(1000); // red for 1 sec (safety)
  setLights(targetLane, false, false, true);
  Serial.print("Lane ");
  Serial.print(targetLane);
  Serial.println(" is now GREEN.");
}

void performGreenToRedTransition(int currentLane) {
  Serial.print("Lane ");
  Serial.print(currentLane);
  Serial.println(": Green -> Yellow -> Red");
  setLights(currentLane, false, true, false);
  delay(3000); // yellow for 3 seconds
  setLights(currentLane, true, false, false);
  Serial.print("Lane ");
  Serial.print(currentLane);
  Serial.println(" is now RED.");
}

void switchToLane(int newActiveLane) {
  if (newActiveLane < 1 || newActiveLane > totalLanes)
    return;
  if (newActiveLane == activeLane)
    return; // already green, nothing to do
  // First, turn off current green (if any)
  if (activeLane >= 1 && activeLane <= totalLanes) {
    performGreenToRedTransition(activeLane);
  }
  delay(100);
  // Then turn on new lane green
  performRedToGreenTransition(newActiveLane);
  activeLane = newActiveLane;
  lastLaneSwitchTime = millis(); // restart green timer
  Serial.print("Now active lane: ");
  Serial.println((int)activeLane);
}

// --- RFID Helpers ---
bool isTagRegistered(MFRC522 &reader) {
  for (int tagIndex = 0; tagIndex < numAuthorizedTags; tagIndex++) {
    bool match = true;
    for (byte i = 0; i < 4; i++) {
      if (reader.uid.uidByte[i] != authorizedUIDs[tagIndex][i]) {
        match = false;
        break;
      }
    }
    if (match)
      return true;
  }
  return false;
}

void printUIDToSerial(MFRC522 &reader) {
  for (byte i = 0; i < reader.uid.size; i++) {
    if (reader.uid.uidByte[i] < 0x10)
      Serial.print("0");
    Serial.print(reader.uid.uidByte[i], HEX);
    if (i < reader.uid.size - 1)
      Serial.print(":");
  }
  Serial.println();
}