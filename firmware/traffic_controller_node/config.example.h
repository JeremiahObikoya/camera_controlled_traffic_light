#pragma once
#include <Arduino.h>

// ============================================================================
// Traffic Controller Node - Configuration Template
// Copy this file to 'config.h' or adjust settings directly for your setup.
// ============================================================================

// --- Wi-Fi Connection to ESP32-CAM SoftAP ---
const char *CAMERA_SSID = "CarDebug";
const char *CAMERA_PASSWORD = "12345678";
const IPAddress CAMERA_IP(192, 168, 4, 1);
const uint16_t UDP_PORT = 4210;

// --- Communication Timers ---
const unsigned long HEARTBEAT_INTERVAL_MS = 500;  // Controller -> camera keepalive
const unsigned long LINK_TIMEOUT_MS = 8000;       // Threshold before fallback to fixed time

// --- Traffic Scheduling Policy ---
const unsigned long LANE_SWITCH_INTERVAL = 30000; // Fixed-time fallback duration per lane (ms)
const unsigned long MIN_GREEN_MS = 10000;         // Camera mode: minimum green time per lane (ms)
const unsigned long MAX_GREEN_MS = 60000;         // Camera mode: maximum green starvation cap (ms)

// --- Shared SPI Bus (for 3x MFRC522 RFID readers) ---
#define SPI_SCK         18
#define SPI_MOSI        23
#define SPI_MISO        19
#define SHARED_RFID_RST 22

// --- RFID Reader Slave-Select (SS / SDA) Pins ---
#define RFID1_SS        4   // Lane 1 reader
#define RFID2_SS        21  // Lane 2 reader
#define RFID3_SS        2   // Lane 3 reader

// --- Traffic Light LED GPIO Pins ---
#define LANE1_RED       12
#define LANE1_YELLOW    15
#define LANE1_GREEN     13

#define LANE2_RED       32
#define LANE2_YELLOW    33
#define LANE2_GREEN     25

#define LANE3_RED       26
#define LANE3_YELLOW    27
#define LANE3_GREEN     14

// --- Authorized Emergency Vehicle RFID UIDs ---
// Replace these with the actual byte UIDs of your authorized emergency keyfobs/cards.
const byte AUTHORIZED_UIDS[][4] = {
    {0xDE, 0xD2, 0x47, 0x06}, // Emergency Tag 1
    {0xD1, 0x60, 0x3E, 0x06}, // Emergency Tag 2
    {0x87, 0x5D, 0x42, 0x06}  // Emergency Tag 3
};
const int NUM_AUTHORIZED_TAGS = sizeof(AUTHORIZED_UIDS) / sizeof(AUTHORIZED_UIDS[0]);

