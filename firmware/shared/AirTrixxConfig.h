#pragma once

#include <Arduino.h>

// AirTrixx network constants.
// All ESP32 boards must use the same ESP-NOW Wi-Fi channel.
static const uint8_t ESPNOW_CHANNEL = 1;

// Use 921600 for lower USB serial latency. If a board/USB cable is unstable,
// set this to 115200 in every firmware target and python_app/config.py.
static const uint32_t AIRTRIXX_SERIAL_BAUD = 921600;

// Keep false on the Antenna during normal operation so the Serial stream stays
// one clean newline-delimited JSON object per line.
static const bool DEBUG_SERIAL = false;

// Paste discovered STA MAC addresses here after flashing firmware/mac_finder.
// Example:
// static uint8_t CAMDOCK_MAC_PLACEHOLDER[6] = {0x24,0x6F,0x28,0xAA,0xBB,0xCC};
static uint8_t ANTENNA_MAC_PLACEHOLDER[6]       = {0x30,0xED,0xA0,0xBB,0x97,0xEC};
static uint8_t CAMDOCK_MAC_PLACEHOLDER[6]       = {0x10,0x20,0xBA,0x4C,0x5B,0xF4};
// static uint8_t WRISTBAND_MAC_PLACEHOLDER[6]     = {0x08,0x92,0x72,0x85,0xAE,0xB8};
static uint8_t WRISTBAND_MAC_PLACEHOLDER[6]     = {0x1C,0xDB,0xD4,0xD4,0xF2,0x0C}; //new
static uint8_t KEYBOARD_MAC_PLACEHOLDER[6]      = {0x10, 0x20, 0xBA, 0x4D, 0xFC, 0xA8};
static uint8_t CHARGING_DOCK_MAC_PLACEHOLDER[6] = {0x30, 0xED, 0xA0, 0xB9, 0xCB, 0xD8};
static uint8_t AUDIODOCK_MAC_PLACEHOLDER[6]     = {0x30, 0xED, 0xA0, 0xB9, 0xCC, 0x3C};
static uint8_t FANS_MAC_PLACEHOLDER[6]          = {0x10, 0x00, 0x3B, 0xAF, 0x94, 0x20};

// Cam Dock hardware pins.
static const int SDA_SERVO = 18;
static const int SCL_SERVO = 21;
static const int SDA_TOF   = 15;
static const int SCL_TOF   = 17;
static const int CAMDOCK_BATTERY_ADC_PIN = 7;

// I2C addresses.
static const uint8_t PCA9685_ADDR = 0x40;
static const uint8_t MUX_ADDR     = 0x70;

// TCA9548A mux channels.
static const uint8_t CH_LEFT_TOF  = 2;
static const uint8_t CH_RIGHT_TOF = 1;

// Keyboard battery divider: GND -> 22k -> ADC pin -> 22k -> LiPo positive.
static const int KEYBOARD_BATTERY_ADC_PIN = 4;
static const float KEYBOARD_BATTERY_DIVIDER_RATIO = 2.0f;
static const float KEYBOARD_BATTERY_EMPTY_V = 3.30f;
static const float KEYBOARD_BATTERY_FULL_V = 4.20f;
static const uint32_t KEYBOARD_BATTERY_REPORT_MS = 20000;

// PCA9685 servo channels.
static const uint8_t CH_R_PAN    = 10;
static const uint8_t CH_R_TILT   = 11;
static const uint8_t CH_CAM_PAN  = 12;
static const uint8_t CH_CAM_TILT = 13;
static const uint8_t CH_L_PAN    = 14;
static const uint8_t CH_L_TILT   = 15;

// Default center pulses for 50 Hz PCA9685 servo outputs.
// 307 ticks is roughly 1500 us at 50 Hz.
static const uint16_t DEFAULT_SERVO_CENTER_TICKS = 226;
static const uint16_t DEFAULT_R_PAN_CENTER       = 226;
static const uint16_t DEFAULT_R_TILT_CENTER      = 226;
static const uint16_t DEFAULT_L_PAN_CENTER       = 242;
static const uint16_t DEFAULT_L_TILT_CENTER      = 226;
static const uint16_t DEFAULT_CAM_PAN_CENTER     = 241;
static const uint16_t DEFAULT_CAM_TILT_CENTER    = 227;

// Cam Dock geometry and one-time calibration placeholders.
// Coordinate system: camera optical center is origin, +X points to the user's
// right in the camera image, +Y points up in the camera image, +Z points
// forward from the dock toward the user.
//
// Hand bracket pan/yaw axis positions relative to the camera lens center.
// These include the measured camera lens offset from the center camera bracket.
static const float DOCK_LEFT_BRACKET_X_MM  = -59.5f;
static const float DOCK_RIGHT_BRACKET_X_MM = 100.5f;
static const float DOCK_LEFT_BRACKET_Y_MM  = 0.0f;
static const float DOCK_RIGHT_BRACKET_Y_MM = 0.0f;
static const float DOCK_LEFT_BRACKET_Z_MM  = -60.0f;
static const float DOCK_RIGHT_BRACKET_Z_MM = -60.0f;

// Tilt axis position relative to the pan/yaw pivot, in the panned bracket
// frame. +Y is upward and +Z is forward. If your measured 45 mm is physically
// below the pan pivot, use -45 mm here.
static const float DOCK_LEFT_TILT_PIVOT_OFFSET_X_MM  = 0.0f;
static const float DOCK_LEFT_TILT_PIVOT_OFFSET_Y_MM  = 0.0f;
static const float DOCK_LEFT_TILT_PIVOT_OFFSET_Z_MM  = 20.0f;
static const float DOCK_RIGHT_TILT_PIVOT_OFFSET_X_MM = 0.0f;
static const float DOCK_RIGHT_TILT_PIVOT_OFFSET_Y_MM = 0.0f;
static const float DOCK_RIGHT_TILT_PIVOT_OFFSET_Z_MM = 20.0f;

// ToF optical center offsets from each tilt axis. Leave at zero until the
// sensor face offset from the tilt pivot is measured.
static const float DOCK_LEFT_TOF_OFFSET_X_MM  = 0.0f;
static const float DOCK_LEFT_TOF_OFFSET_Y_MM  = 0.0f;
static const float DOCK_LEFT_TOF_OFFSET_Z_MM  = 25.0f;
static const float DOCK_RIGHT_TOF_OFFSET_X_MM = 0.0f;
static const float DOCK_RIGHT_TOF_OFFSET_Y_MM = 0.0f;
static const float DOCK_RIGHT_TOF_OFFSET_Z_MM = 25.0f;

// Camera and tracking placeholders. These are mirrored in python_app/config.py
// because the PC app owns MediaPipe ray math.
static const float DOCK_CAMERA_HORIZONTAL_FOV_DEG = 70.0f;
static const float DOCK_CAMERA_VERTICAL_FOV_DEG   = 43.0f;
static const float DOCK_INITIAL_HAND_DISTANCE_MM  = 700.0f;
static const float DOCK_MIN_VALID_TOF_MM          = 80.0f;
static const float DOCK_MAX_VALID_TOF_MM          = 2000.0f;
static const bool DOCK_USE_STARTUP_USER_DISTANCE  = true;
static const float DOCK_STARTUP_DISTANCE_LIVE_WEIGHT = 0.35f;
static const uint8_t DOCK_TRACKING_FRAME_SKIP = 1;
static const uint8_t DOCK_PREVIEW_FPS = 10;
static const bool DOCK_FACE_DETECTION_ENABLED_AFTER_CENTERING = false;
static const uint16_t DOCK_CAMERA_WIDTH = 424;
static const uint16_t DOCK_CAMERA_HEIGHT = 240;
static const float DOCK_TRACKING_LATENCY_MS       = 50.0f;

// Servo angle conversion placeholders. Tune these once per build after finding
// each servo's center pulse. A common 500-2500 us / 180 degree servo is about
// 2.27 PCA9685 ticks per degree at 50 Hz.
static const float DOCK_PAN_TICKS_PER_DEGREE  = 2.25f;
static const float DOCK_TILT_TICKS_PER_DEGREE = 2.25f;
static const float DOCK_CAM_PAN_SIGN          = -1.0f;
static const float DOCK_CAM_TILT_SIGN         = -1.0f;
static const float DOCK_CAM_PAN_OFFSET_DEG    = 0.0f;
static const float DOCK_CAM_TILT_OFFSET_DEG   = 0.0f;
static const float DOCK_RIGHT_PAN_SIGN        = -1.0f;
static const float DOCK_RIGHT_TILT_SIGN       = -1.0f;
static const float DOCK_LEFT_PAN_SIGN         = -1.0f;
static const float DOCK_LEFT_TILT_SIGN        = -1.0f;
static const float DOCK_RIGHT_PAN_OFFSET_DEG  = 27.17f;
static const float DOCK_RIGHT_TILT_OFFSET_DEG = 0.0f;
static const float DOCK_LEFT_PAN_OFFSET_DEG   = -6.93f;
static const float DOCK_LEFT_TILT_OFFSET_DEG  = 0.0f;

// Runtime timing.
static const uint16_t ANTENNA_JSON_HZ = 30;
static const uint16_t CAMDOCK_REPORT_HZ = 30;
static const uint16_t WRISTBAND_REPORT_HZ = 50;
static const uint16_t FANS_REPORT_HZ = 2;
static const uint16_t KEYBOARD_REPORT_HZ = 30;
static const uint16_t CHARGING_DOCK_REPORT_HZ = 2;
static const uint32_t DEVICE_TIMEOUT_MS = 1000;
