#pragma once

// ============================================================================
// Camera Vision Node - Configuration Template
// Copy this file to 'config.h' or adjust settings directly for your setup.
// ============================================================================

// --- SoftAP Wi-Fi Settings ---
#define AP_SSID        "CarDebug"
#define AP_PASSWORD    "12345678"      // Must be at least 8 characters
#define AP_CHANNEL     1
#define AP_MAX_CLIENTS 4               // Traffic controller + debug web clients

// --- Controller UDP Link ---
#define UDP_PORT              4210
#define CONTROLLER_TIMEOUT_MS 3000     // Heartbeat timeout threshold

// --- Region of Interest (ROI) Mapping ---
// Names must match the ROI labels exported from tools/roi_tool.html
static const char *LANE_ROI_NAMES[3] = {"ROI 1", "ROI 2", "ROI 3"};

// --- Image Preprocessing ---
// 1 = centre-crop to model aspect ratio then scale; 0 = squash full 320x240 frame
#define RESIZE_MODE_CROP 1

// --- ROI Point Anchor ---
// 1 = bottom-centre of detection box; 0 = centre of detection box
#define ROI_ANCHOR_BOTTOM_CENTER 1

// --- Detection Filters ---
#define DEFAULT_THRESHOLD              0.55f
#define TRAFFIC_MIN_AVERAGE_CONFIDENCE 0.80f
#define TRAFFIC_REQUIRED_FRAMES        3

// --- Vehicle Bounding-Box Merging & Splitting Limits ---
#define MAX_CAR_BOX_W     26          // Max expected width (px) for single car
#define MAX_CAR_BOX_H     30          // Max expected height (px) for single car
#define MAX_CAR_BOX_AREA  975         // Area threshold to trigger box splitting
#define MAX_CAR_SPLIT     4           // Max vehicles to split out of merged box

