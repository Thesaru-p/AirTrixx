#pragma once

#include <Arduino.h>
#include <stdint.h>

#define AIRTRIXX_PROTOCOL_VERSION 1

#if defined(__GNUC__)
#define AIRTRIXX_PACKED __attribute__((packed))
#else
#define AIRTRIXX_PACKED
#endif

enum AirTrixxDeviceId : uint8_t {
  DEVICE_ANTENNA       = 1,
  DEVICE_CAMDOCK       = 2,
  DEVICE_WRISTBAND     = 3,
  DEVICE_KEYBOARD      = 4,
  DEVICE_CHARGING_DOCK = 5,
  DEVICE_AUDIODOCK     = 6,
  DEVICE_FANS          = 7
};

enum AirTrixxMessageType : uint8_t {
  MSG_WRISTBAND_DATA = 1,
  MSG_CAMDOCK_DATA   = 2,
  MSG_SERVO_COMMAND  = 3,
  MSG_HEARTBEAT      = 4,
  MSG_ACK            = 5,
  MSG_BATTERY_STATUS = 6,
  MSG_OTA_START      = 7,
  MSG_FAN_STATUS     = 8,
  MSG_FAN_COMMAND    = 9,
  MSG_OTA_CHUNK      = 10,
  MSG_KEYBOARD_TOF   = 11,
  MSG_AUDIODOCK_DATA = 12,
  MSG_AUDIODOCK_AUDIO_CHUNK = 13,
  MSG_AUDIODOCK_TRANSCRIPT = 14
};

static const uint8_t AIRTRIXX_OTA_SSID_MAX = 32;
static const uint8_t AIRTRIXX_OTA_PASSWORD_MAX = 64;
static const uint8_t AIRTRIXX_OTA_URL_MAX = 96;
static const uint8_t AIRTRIXX_OTA_MD5_MAX = 32;
static const uint8_t AIRTRIXX_OTA_CHUNK_BYTES = 32;

enum AirTrixxOtaChunkField : uint8_t {
  OTA_FIELD_RESET    = 0,
  OTA_FIELD_SSID     = 1,
  OTA_FIELD_PASSWORD = 2,
  OTA_FIELD_URL      = 3,
  OTA_FIELD_MD5      = 4,
  OTA_FIELD_COMMIT   = 255
};

enum AirTrixxActivePair : uint8_t {
  ACTIVE_PAIR_NONE   = 0,
  ACTIVE_PAIR_RIGHT  = 1,
  ACTIVE_PAIR_LEFT   = 2,
  ACTIVE_PAIR_CAMERA = 3,
  ACTIVE_PAIR_HANDS  = 4
};

enum AirTrixxServoMask : uint8_t {
  SERVO_MASK_R_PAN    = 1 << 0,
  SERVO_MASK_R_TILT   = 1 << 1,
  SERVO_MASK_L_PAN    = 1 << 2,
  SERVO_MASK_L_TILT   = 1 << 3,
  SERVO_MASK_CAM_PAN  = 1 << 4,
  SERVO_MASK_CAM_TILT = 1 << 5
};

// Scaling used by compact ESP-NOW packets:
// - accelerometer: milligravity (mg). Convert to m/s^2 with value * 0.00980665.
// - gyro: millidegrees/second (mdps). Convert to deg/s with value * 0.001.
// - pitch/roll: centidegrees. Convert to degrees with value * 0.01.
static const float AIRTRIXX_ACCEL_MG_TO_MPS2 = 0.00980665f;
static const float AIRTRIXX_GYRO_MDPS_TO_DPS = 0.001f;
static const float AIRTRIXX_CDEG_TO_DEG      = 0.01f;

struct AIRTRIXX_PACKED AirTrixxPacketHeader {
  uint8_t protocol_version;
  uint8_t msg_type;
  uint8_t device_id;
  uint16_t sequence;
  uint32_t t_ms;
  // Legacy one-bit flag. Battery level telemetry is sent separately with
  // MSG_BATTERY_STATUS so high-rate motion packets do not carry battery data.
  uint8_t battery_low;
};

struct AIRTRIXX_PACKED WristbandDataPacket {
  AirTrixxPacketHeader header;
  int16_t accel_mg_x;
  int16_t accel_mg_y;
  int16_t accel_mg_z;
  int32_t gyro_mdps_x;
  int32_t gyro_mdps_y;
  int32_t gyro_mdps_z;
  int16_t pitch_cdeg;
  int16_t roll_cdeg;
};

struct AIRTRIXX_PACKED BatteryStatusPacket {
  AirTrixxPacketHeader header;
  uint16_t battery_mv;
  uint8_t battery_percent;
  uint8_t battery_valid;
  uint16_t battery_adc_raw;
};

struct AIRTRIXX_PACKED CamDockDataPacket {
  AirTrixxPacketHeader header;
  uint16_t left_tof_mm;
  uint16_t right_tof_mm;
  uint16_t battery_mv;
  uint16_t battery_adc_raw;
  uint8_t battery_percent;
  uint8_t battery_valid;
  uint8_t active_target;
};

struct AIRTRIXX_PACKED ServoCommandPacket {
  AirTrixxPacketHeader header;
  uint8_t active_pair;
  uint8_t active_mask;
  uint8_t disable_unused;
  uint16_t r_pan;
  uint16_t r_tilt;
  uint16_t l_pan;
  uint16_t l_tilt;
  uint16_t cam_pan;
  uint16_t cam_tilt;
};

struct AIRTRIXX_PACKED FanStatusPacket {
  AirTrixxPacketHeader header;
  uint8_t fan_on;
  uint8_t temp1_valid;
  uint8_t temp2_valid;
  int16_t temp1_centi_c;
  int16_t temp2_centi_c;
  uint16_t last_command_sequence;
  uint16_t battery_mv;
  uint16_t battery_adc_raw;
  uint8_t battery_percent;
  uint8_t battery_valid;
};

struct AIRTRIXX_PACKED FanCommandPacket {
  AirTrixxPacketHeader header;
  uint8_t fan_on;
};

struct AIRTRIXX_PACKED KeyboardTofPacket {
  AirTrixxPacketHeader header;
  uint16_t distance_mm_1;
  uint16_t distance_mm_2;
  uint16_t distance_mm_3;
  uint16_t distance_mm_4;
  uint8_t valid_1;
  uint8_t valid_2;
  uint8_t valid_3;
  uint8_t valid_4;
};

struct AIRTRIXX_PACKED AudioDockDataPacket {
  AirTrixxPacketHeader header;
  uint8_t clap_detected;
  uint8_t clap_type;     // 1 = Single, 2 = Double
  uint32_t audio_size;
};

struct AIRTRIXX_PACKED AudioDockChunkPacket {
  AirTrixxPacketHeader header;
  uint32_t chunk_index;
  uint16_t chunk_len;
  uint8_t data[200];
};

struct AIRTRIXX_PACKED AudioDockTranscriptPacket {
  AirTrixxPacketHeader header;
  char transcript[128];
};

struct AIRTRIXX_PACKED HeartbeatPacket {
  AirTrixxPacketHeader header;
};

struct AIRTRIXX_PACKED AckPacket {
  AirTrixxPacketHeader header;
  uint8_t acked_msg_type;
  uint16_t acked_sequence;
  uint8_t status;
};

struct AIRTRIXX_PACKED OtaStartPacket {
  AirTrixxPacketHeader header;
  char ssid[AIRTRIXX_OTA_SSID_MAX + 1];
  char password[AIRTRIXX_OTA_PASSWORD_MAX + 1];
  char url[AIRTRIXX_OTA_URL_MAX + 1];
  char md5[AIRTRIXX_OTA_MD5_MAX + 1];
};

struct AIRTRIXX_PACKED OtaChunkPacket {
  AirTrixxPacketHeader header;
  uint8_t field_id;
  uint8_t offset;
  uint8_t total_len;
  uint8_t chunk_len;
  char data[AIRTRIXX_OTA_CHUNK_BYTES];
};

inline const char *activePairToString(uint8_t active_pair) {
  switch (active_pair) {
    case ACTIVE_PAIR_RIGHT: return "right";
    case ACTIVE_PAIR_LEFT: return "left";
    case ACTIVE_PAIR_CAMERA: return "camera";
    case ACTIVE_PAIR_HANDS: return "hands";
    default: return "none";
  }
}

inline uint8_t maskForActivePair(uint8_t active_pair) {
  switch (active_pair) {
    case ACTIVE_PAIR_RIGHT:
      return SERVO_MASK_R_PAN | SERVO_MASK_R_TILT;
    case ACTIVE_PAIR_LEFT:
      return SERVO_MASK_L_PAN | SERVO_MASK_L_TILT;
    case ACTIVE_PAIR_CAMERA:
      return SERVO_MASK_CAM_PAN | SERVO_MASK_CAM_TILT;
    case ACTIVE_PAIR_HANDS:
      return SERVO_MASK_R_PAN | SERVO_MASK_R_TILT | SERVO_MASK_L_PAN | SERVO_MASK_L_TILT;
    default:
      return 0;
  }
}

inline void fillHeader(AirTrixxPacketHeader &header,
                       uint8_t msg_type,
                       uint8_t device_id,
                       uint16_t sequence,
                       uint32_t t_ms,
                       bool battery_low) {
  header.protocol_version = AIRTRIXX_PROTOCOL_VERSION;
  header.msg_type = msg_type;
  header.device_id = device_id;
  header.sequence = sequence;
  header.t_ms = t_ms;
  header.battery_low = battery_low ? 1 : 0;
}
