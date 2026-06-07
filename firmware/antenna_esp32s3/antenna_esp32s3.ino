#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

static SemaphoreHandle_t serialMutex = NULL;

struct LatestWristband {
  bool seen = false;
  uint32_t received_ms = 0;
  WristbandDataPacket packet = {};
};

struct LatestBatteryStatus {
  bool seen = false;
  uint32_t received_ms = 0;
  BatteryStatusPacket packet = {};
};

struct LatestCamDock {
  bool seen = false;
  uint32_t received_ms = 0;
  CamDockDataPacket packet = {};
};

struct LatestFans {
  bool seen = false;
  uint32_t received_ms = 0;
  FanStatusPacket packet = {};
};

struct LatestKeyboard {
  bool seen = false;
  uint32_t received_ms = 0;
  KeyboardTofPacket packet = {};
};

struct LatestChargingDock {
  bool seen = false;
  uint32_t received_ms = 0;
  ChargingDockStatusPacket packet = {};
};

struct LatestAudioDock {
  bool seen = false;
  uint32_t received_ms = 0;
  AudioDockDataPacket packet = {};
};

struct AIRTRIXX_PACKED LegacyFanStatusPacket {
  AirTrixxPacketHeader header;
  uint8_t fan_on;
  uint8_t temp1_valid;
  uint8_t temp2_valid;
  int16_t temp1_centi_c;
  int16_t temp2_centi_c;
  uint16_t last_command_sequence;
};

static LatestWristband latestWristband;
static LatestBatteryStatus latestWristbandBattery;
static LatestCamDock latestCamDock;
static LatestFans latestFans;
static LatestKeyboard latestKeyboard;
static LatestChargingDock latestChargingDock;
static LatestAudioDock latestAudioDock;
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t antennaJsonSequence = 0;
static uint16_t servoCommandSequence = 0;
static uint16_t otaCommandSequence = 0;
static uint16_t fanCommandSequence = 0;
static uint32_t lastJsonMs = 0;
static bool isStreamingAudioDock = false;
static uint32_t lastAudioDockChunkMs = 0;
static const uint32_t AUDIODOCK_STREAM_TIMEOUT_MS = 6000;

static QueueHandle_t audioChunkQueue = NULL;

static char serialLine[768];
static size_t serialLineLen = 0;
static const uint32_t BATTERY_STATUS_STALE_MS = 11UL * 60UL * 1000UL;
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.

void debugPrintln(const String &message) {
  if (DEBUG_SERIAL) {
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println(message);
      xSemaphoreGive(serialMutex);
    }
  }
}

void configureWiFiChannel() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
}

bool addEspNowPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    debugPrintln("ESP-NOW add peer failed: " + String(result));
    return false;
  }
  return true;
}

uint8_t parseActivePair(const String &value) {
  if (value == "right") {
    return ACTIVE_PAIR_RIGHT;
  }
  if (value == "left") {
    return ACTIVE_PAIR_LEFT;
  }
  if (value == "camera") {
    return ACTIVE_PAIR_CAMERA;
  }
  if (value == "hands" || value == "both") {
    return ACTIVE_PAIR_HANDS;
  }
  if (value == "dock") {
    return ACTIVE_PAIR_DOCK;
  }
  return ACTIVE_PAIR_NONE;
}

bool extractStringField(const String &json, const char *key, String &out) {
  String pattern = "\"" + String(key) + "\"";
  int keyIndex = json.indexOf(pattern);
  if (keyIndex < 0) {
    return false;
  }
  int colon = json.indexOf(':', keyIndex + pattern.length());
  if (colon < 0) {
    return false;
  }
  int startQuote = json.indexOf('"', colon + 1);
  if (startQuote < 0) {
    return false;
  }
  int endQuote = json.indexOf('"', startQuote + 1);
  if (endQuote < 0) {
    return false;
  }
  out = json.substring(startQuote + 1, endQuote);
  return true;
}

bool extractBoolField(const String &json, const char *key, bool &out) {
  String pattern = "\"" + String(key) + "\"";
  int keyIndex = json.indexOf(pattern);
  if (keyIndex < 0) {
    return false;
  }
  int colon = json.indexOf(':', keyIndex + pattern.length());
  if (colon < 0) {
    return false;
  }
  int valueStart = colon + 1;
  while (valueStart < json.length() && isspace(json[valueStart])) {
    valueStart++;
  }
  if (json.startsWith("true", valueStart)) {
    out = true;
    return true;
  }
  if (json.startsWith("false", valueStart)) {
    out = false;
    return true;
  }
  return false;
}

bool extractUInt16Field(const String &json, const char *key, uint16_t &out) {
  String pattern = "\"" + String(key) + "\"";
  int keyIndex = json.indexOf(pattern);
  if (keyIndex < 0) {
    return false;
  }
  int colon = json.indexOf(':', keyIndex + pattern.length());
  if (colon < 0) {
    return false;
  }

  int valueStart = colon + 1;
  while (valueStart < json.length() && isspace(json[valueStart])) {
    valueStart++;
  }
  int valueEnd = valueStart;
  while (valueEnd < json.length() && isDigit(json[valueEnd])) {
    valueEnd++;
  }
  if (valueEnd == valueStart) {
    return false;
  }

  long value = json.substring(valueStart, valueEnd).toInt();
  if (value < 0) {
    value = 0;
  }
  if (value > 4095) {
    value = 4095;
  }
  out = static_cast<uint16_t>(value);
  return true;
}

void copyStringToPacketField(char *field, size_t fieldSize, const String &value) {
  if (field == nullptr || fieldSize == 0) {
    return;
  }
  size_t len = min(static_cast<size_t>(value.length()), fieldSize - 1);
  memcpy(field, value.c_str(), len);
  field[len] = '\0';
}

size_t boundedCStringLength(const char *value, size_t maxLen) {
  size_t len = 0;
  while (len < maxLen && value[len] != '\0') {
    len++;
  }
  return len;
}

void sendServoCommandToCamDock(const ServoCommandPacket &packet) {
  esp_err_t result = esp_now_send(CAMDOCK_MAC_PLACEHOLDER,
                                  reinterpret_cast<const uint8_t *>(&packet),
                                  sizeof(packet));
  if (result != ESP_OK) {
    debugPrintln("ESP-NOW servo send failed: " + String(result));
  }
}

void sendOtaStartToTarget(const OtaStartPacket &packet, const uint8_t mac[6], const char *label) {
  esp_err_t result = esp_now_send(mac,
                                  reinterpret_cast<const uint8_t *>(&packet),
                                  sizeof(packet));
  if (result != ESP_OK) {
    debugPrintln("ESP-NOW " + String(label) + " OTA send failed: " + String(result));
  }
}

void sendOtaStartToFans(const OtaStartPacket &packet) {
  auto sendChunk = [](OtaChunkPacket &chunk, uint8_t repeats) {
    for (uint8_t attempt = 0; attempt < repeats; ++attempt) {
      esp_err_t result = esp_now_send(ESPNOW_BROADCAST_MAC,
                                      reinterpret_cast<const uint8_t *>(&chunk),
                                      sizeof(chunk));
      if (result != ESP_OK) {
        debugPrintln("ESP-NOW fans OTA chunk send failed: " + String(result));
      }
      delay(30);
    }
  };

  auto sendControl = [&](uint8_t fieldId, uint8_t repeats) {
    OtaChunkPacket chunk = {};
    fillHeader(chunk.header,
               MSG_OTA_CHUNK,
               DEVICE_ANTENNA,
               ++otaCommandSequence,
               millis(),
               false);
    chunk.field_id = fieldId;
    sendChunk(chunk, repeats);
  };

  auto sendField = [&](uint8_t fieldId, const char *value, size_t maxLen) {
    size_t totalLen = boundedCStringLength(value, maxLen);
    size_t offset = 0;
    do {
      OtaChunkPacket chunk = {};
      fillHeader(chunk.header,
                 MSG_OTA_CHUNK,
                 DEVICE_ANTENNA,
                 ++otaCommandSequence,
                 millis(),
                 false);
      chunk.field_id = fieldId;
      chunk.offset = static_cast<uint8_t>(offset);
      chunk.total_len = static_cast<uint8_t>(totalLen);
      size_t remaining = totalLen > offset ? totalLen - offset : 0;
      size_t chunkLen = min(static_cast<size_t>(AIRTRIXX_OTA_CHUNK_BYTES), remaining);
      chunk.chunk_len = static_cast<uint8_t>(chunkLen);
      if (chunkLen > 0) {
        memcpy(chunk.data, value + offset, chunkLen);
      }
      sendChunk(chunk, 2);
      offset += chunkLen;
    } while (offset < totalLen);
  };

  sendControl(OTA_FIELD_RESET, 4);
  sendField(OTA_FIELD_SSID, packet.ssid, AIRTRIXX_OTA_SSID_MAX);
  sendField(OTA_FIELD_PASSWORD, packet.password, AIRTRIXX_OTA_PASSWORD_MAX);
  sendField(OTA_FIELD_URL, packet.url, AIRTRIXX_OTA_URL_MAX);
  sendField(OTA_FIELD_MD5, packet.md5, AIRTRIXX_OTA_MD5_MAX);
  sendControl(OTA_FIELD_COMMIT, 8);
}

void sendFanCommandToFans(const FanCommandPacket &packet) {
  esp_err_t result = esp_now_send(ESPNOW_BROADCAST_MAC,
                                  reinterpret_cast<const uint8_t *>(&packet),
                                  sizeof(packet));
  if (result != ESP_OK) {
    debugPrintln("ESP-NOW fan command send failed: " + String(result));
  }
}

void handleOtaJsonCommand(const String &line) {
  String target;
  if (!extractStringField(line, "target", target)) {
    debugPrintln("OTA command missing target");
    return;
  }
  bool targetWristband = target == "wristband";
  bool targetFans = target == "fans" || target == "fan";
  bool targetCamDock = target == "camdock" || target == "cam_dock" || target == "camera_dock";
  if (!targetWristband && !targetFans && !targetCamDock) {
    debugPrintln("Ignoring unsupported OTA target: " + target);
    return;
  }

  String ssid;
  String password;
  String url;
  String md5;
  if (!extractStringField(line, "ssid", ssid) || ssid.length() == 0) {
    debugPrintln("OTA command missing ssid");
    return;
  }
  if (!extractStringField(line, "password", password)) {
    password = "";
  }
  if (!extractStringField(line, "url", url) || url.length() == 0) {
    debugPrintln("OTA command missing url");
    return;
  }
  extractStringField(line, "md5", md5);

  OtaStartPacket packet = {};
  fillHeader(packet.header,
             MSG_OTA_START,
             DEVICE_ANTENNA,
             ++otaCommandSequence,
             millis(),
             false);
  copyStringToPacketField(packet.ssid, sizeof(packet.ssid), ssid);
  copyStringToPacketField(packet.password, sizeof(packet.password), password);
  copyStringToPacketField(packet.url, sizeof(packet.url), url);
  copyStringToPacketField(packet.md5, sizeof(packet.md5), md5);
  if (targetWristband) {
    sendOtaStartToTarget(packet, WRISTBAND_MAC_PLACEHOLDER, "wristband");
  } else if (targetCamDock) {
    sendOtaStartToTarget(packet, CAMDOCK_MAC_PLACEHOLDER, "camdock");
  } else {
    sendOtaStartToFans(packet);
  }
}

void handleFanJsonCommand(const String &line) {
  String target;
  if (!extractStringField(line, "target", target) || target != "fans") {
    debugPrintln("Ignoring non-fans command target");
    return;
  }

  bool fanOn = false;
  if (!extractBoolField(line, "fan_on", fanOn)) {
    debugPrintln("Fans command missing fan_on");
    return;
  }

  FanCommandPacket packet = {};
  fillHeader(packet.header,
             MSG_FAN_COMMAND,
             DEVICE_ANTENNA,
             ++fanCommandSequence,
             millis(),
             false);
  packet.fan_on = fanOn ? 1 : 0;
  sendFanCommandToFans(packet);
}

bool sendAudioDockTextPacket(const String &text, const char *statusPrefix, const String &statusValue = "");

void handleSerialJsonCommand(const String &line) {
  String cmd;
  if (!extractStringField(line, "cmd", cmd)) {
    debugPrintln("Serial command missing cmd");
    return;
  }
  if (cmd == "ota") {
    handleOtaJsonCommand(line);
    return;
  }
  if (cmd == "fans" || cmd == "fan") {
    handleFanJsonCommand(line);
    return;
  }
  if (cmd == "audiodock") {
    String control;
    bool hasControl = extractStringField(line, "control", control) ||
                      extractStringField(line, "action", control);
    if (hasControl) {
      control.trim();
      control.toLowerCase();

      String commandText;
      if (control == "ledtest" || control == "led_test" || control == "led") {
        commandText = "__CMD:LEDTEST__";
      } else if (control == "speakertest" || control == "speaker_test" ||
                 control == "speaker" || control == "spktest") {
        commandText = "__CMD:SPEAKERTEST__";
      } else {
        debugPrintln("Unsupported audiodock control: " + control);
        return;
      }

      sendAudioDockTextPacket(commandText, "ANTENNA_AUDIODOCK_CONTROL:", control);
      return;
    }

    String transcript;
    if (extractStringField(line, "transcript", transcript)) {
      sendAudioDockTextPacket(transcript, "ANTENNA_AUDIODOCK_TRANSCRIPT:");
    }
    return;
  }
  if (cmd != "servo") {
    debugPrintln("Unsupported command: " + cmd);
    return;
  }

  String target;
  if (!extractStringField(line, "target", target) || target != "camdock") {
    debugPrintln("Ignoring non-camdock servo target");
    return;
  }

  String activePairString;
  if (!extractStringField(line, "active_pair", activePairString)) {
    activePairString = "none";
  }

  bool disableUnused = true;
  extractBoolField(line, "disable_unused", disableUnused);

  ServoCommandPacket packet = {};
  fillHeader(packet.header,
             MSG_SERVO_COMMAND,
             DEVICE_ANTENNA,
             ++servoCommandSequence,
             millis(),
             false);
  packet.active_pair = parseActivePair(activePairString);
  packet.active_mask = maskForActivePair(packet.active_pair);
  packet.disable_unused = disableUnused ? 1 : 0;

  // Packed struct fields may be unaligned, so parse into aligned locals before
  // assigning into the packet.
  uint16_t rPan = 0;
  uint16_t rTilt = 0;
  uint16_t lPan = 0;
  uint16_t lTilt = 0;
  uint16_t camPan = 0;
  uint16_t camTilt = 0;
  extractUInt16Field(line, "r_pan", rPan);
  extractUInt16Field(line, "r_tilt", rTilt);
  extractUInt16Field(line, "l_pan", lPan);
  extractUInt16Field(line, "l_tilt", lTilt);
  extractUInt16Field(line, "cam_pan", camPan);
  extractUInt16Field(line, "cam_tilt", camTilt);
  packet.r_pan = rPan;
  packet.r_tilt = rTilt;
  packet.l_pan = lPan;
  packet.l_tilt = lTilt;
  packet.cam_pan = camPan;
  packet.cam_tilt = camTilt;

  sendServoCommandToCamDock(packet);
}

void pumpSerialCommands() {
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    while (Serial.available() > 0) {
      char c = static_cast<char>(Serial.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        serialLine[serialLineLen] = '\0';
        if (serialLineLen > 0) {
          String line(serialLine);
          line.trim();
          if (line.length() > 0) {
            // Unlock mutex before calling command handler to prevent recursive deadlocks
            xSemaphoreGive(serialMutex);
            handleSerialJsonCommand(line);
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) {
              serialLineLen = 0;
              return;
            }
          }
        }
        serialLineLen = 0;
        continue;
      }
      if (serialLineLen < sizeof(serialLine) - 1) {
        serialLine[serialLineLen++] = c;
      } else {
        serialLineLen = 0;
        xSemaphoreGive(serialMutex);
        debugPrintln("Serial command too long; dropped");
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;
      }
    }
    xSemaphoreGive(serialMutex);
  }
}

bool sendAudioDockTextPacket(const String &text, const char *statusPrefix, const String &statusValue) {
  uint8_t okCount = 0;
  esp_err_t lastResult = ESP_FAIL;

  for (uint8_t attempt = 0; attempt < 8; attempt++) {
    AudioDockTranscriptPacket packet = {};
    fillHeader(packet.header,
               MSG_AUDIODOCK_TRANSCRIPT,
               DEVICE_ANTENNA,
               ++antennaJsonSequence,
               millis(),
               false);
    copyStringToPacketField(packet.transcript, sizeof(packet.transcript), text);
    lastResult = esp_now_send(ESPNOW_BROADCAST_MAC,
                              reinterpret_cast<const uint8_t *>(&packet),
                              sizeof(packet));
    if (lastResult == ESP_OK) {
      okCount++;
    }
    delay(35);
  }

  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(statusPrefix);
    if (statusValue.length() > 0) {
      Serial.print(statusValue);
      Serial.print(",");
    }
    Serial.print(okCount > 0 ? "sent" : "failed");
    Serial.print(",ok=");
    Serial.print(okCount);
    Serial.print(",last=");
    Serial.println(lastResult);
    xSemaphoreGive(serialMutex);
  }

  return okCount > 0;
}

void handleIncomingPacket(const uint8_t *data, int len) {
  if (len < static_cast<int>(sizeof(AirTrixxPacketHeader))) {
    return;
  }

  AirTrixxPacketHeader header = {};
  memcpy(&header, data, sizeof(header));
  if (header.protocol_version != AIRTRIXX_PROTOCOL_VERSION) {
    return;
  }

  if (header.msg_type == MSG_WRISTBAND_DATA && len == static_cast<int>(sizeof(WristbandDataPacket))) {
    WristbandDataPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    portENTER_CRITICAL(&stateMux);
    latestWristband.packet = packet;
    latestWristband.seen = true;
    latestWristband.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_BATTERY_STATUS &&
             header.device_id == DEVICE_WRISTBAND &&
             len == static_cast<int>(sizeof(BatteryStatusPacket))) {
    BatteryStatusPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    portENTER_CRITICAL(&stateMux);
    latestWristbandBattery.packet = packet;
    latestWristbandBattery.seen = true;
    latestWristbandBattery.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_CAMDOCK_DATA && len == static_cast<int>(sizeof(CamDockDataPacket))) {
    CamDockDataPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    portENTER_CRITICAL(&stateMux);
    latestCamDock.packet = packet;
    latestCamDock.seen = true;
    latestCamDock.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_KEYBOARD_TOF && len == static_cast<int>(sizeof(KeyboardTofPacket))) {
    KeyboardTofPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.header.device_id != DEVICE_KEYBOARD) {
      return;
    }
    portENTER_CRITICAL(&stateMux);
    latestKeyboard.packet = packet;
    latestKeyboard.seen = true;
    latestKeyboard.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_CHARGING_DOCK_STATUS &&
             len == static_cast<int>(sizeof(ChargingDockStatusPacket))) {
    ChargingDockStatusPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.header.device_id != DEVICE_CHARGING_DOCK) {
      return;
    }
    portENTER_CRITICAL(&stateMux);
    latestChargingDock.packet = packet;
    latestChargingDock.seen = true;
    latestChargingDock.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_FAN_STATUS && len == static_cast<int>(sizeof(FanStatusPacket))) {
    FanStatusPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.header.device_id != DEVICE_FANS) {
      return;
    }
    portENTER_CRITICAL(&stateMux);
    latestFans.packet = packet;
    latestFans.seen = true;
    latestFans.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_FAN_STATUS &&
             len == static_cast<int>(sizeof(LegacyFanStatusPacket))) {
    LegacyFanStatusPacket legacy = {};
    memcpy(&legacy, data, sizeof(legacy));
    if (legacy.header.device_id != DEVICE_FANS) {
      return;
    }
    FanStatusPacket packet = {};
    packet.header = legacy.header;
    packet.fan_on = legacy.fan_on;
    packet.temp1_valid = legacy.temp1_valid;
    packet.temp2_valid = legacy.temp2_valid;
    packet.temp1_centi_c = legacy.temp1_centi_c;
    packet.temp2_centi_c = legacy.temp2_centi_c;
    packet.last_command_sequence = legacy.last_command_sequence;
    packet.battery_valid = 0;
    portENTER_CRITICAL(&stateMux);
    latestFans.packet = packet;
    latestFans.seen = true;
    latestFans.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
  } else if (header.msg_type == MSG_AUDIODOCK_DATA && len == static_cast<int>(sizeof(AudioDockDataPacket))) {
    AudioDockDataPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.header.device_id != DEVICE_AUDIODOCK) {
      return;
    }
    portENTER_CRITICAL(&stateMux);
    latestAudioDock.packet = packet;
    latestAudioDock.seen = true;
    latestAudioDock.received_ms = millis();
    portEXIT_CRITICAL(&stateMux);
    
    isStreamingAudioDock = true;
    lastAudioDockChunkMs = millis();
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.printf("AUDIODOCK_TRIGGER:%d,%d\n", packet.clap_type, packet.audio_size);
      xSemaphoreGive(serialMutex);
    }
  } else if (header.msg_type == MSG_AUDIODOCK_AUDIO_CHUNK && len == static_cast<int>(sizeof(AudioDockChunkPacket))) {
    AudioDockChunkPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.header.device_id != DEVICE_AUDIODOCK) {
      return;
    }
    
    isStreamingAudioDock = true;
    lastAudioDockChunkMs = millis();
    
    if (audioChunkQueue != NULL) {
      xQueueSend(audioChunkQueue, &packet, 0);
    }
  } else if (DEBUG_SERIAL) {
    debugPrintln("Unexpected ESP-NOW packet type/size");
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info;
  handleIncomingPacket(incomingData, len);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  (void)mac;
  handleIncomingPacket(incomingData, len);
}
#endif

void printWristbandJson(const LatestWristband &snapshot,
                        const LatestBatteryStatus &batterySnapshot,
                        uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  bool batteryKnown = batterySnapshot.seen;
  bool batteryFresh = batteryKnown && (nowMs - batterySnapshot.received_ms <= BATTERY_STATUS_STALE_MS);
  bool batteryValid = batteryFresh && batterySnapshot.packet.battery_valid != 0;
  Serial.print("\"wristband\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"battery_level\":");
  if (batteryValid) {
    Serial.print(batterySnapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery_voltage\":");
  if (batteryValid) {
    Serial.print(batterySnapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery\":{\"status\":\"");
  if (!batteryKnown) {
    Serial.print("unknown");
  } else if (!batteryFresh) {
    Serial.print("stale");
  } else if (!batteryValid) {
    Serial.print("invalid");
  } else {
    Serial.print("ok");
  }
  Serial.print("\",\"percent\":");
  if (batteryValid) {
    Serial.print(batterySnapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"voltage_v\":");
  if (batteryValid) {
    Serial.print(batterySnapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"adc_raw\":");
  if (batteryKnown) {
    Serial.print(batterySnapshot.packet.battery_adc_raw);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"age_ms\":");
  if (batteryKnown) {
    Serial.print(nowMs - batterySnapshot.received_ms);
  } else {
    Serial.print("null");
  }
  Serial.print("}");
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(snapshot.packet.header.t_ms);
  } else {
    Serial.print("null");
  }

  if (ok) {
    float ax = snapshot.packet.accel_mg_x * AIRTRIXX_ACCEL_MG_TO_MPS2;
    float ay = snapshot.packet.accel_mg_y * AIRTRIXX_ACCEL_MG_TO_MPS2;
    float az = snapshot.packet.accel_mg_z * AIRTRIXX_ACCEL_MG_TO_MPS2;
    float gx = snapshot.packet.gyro_mdps_x * AIRTRIXX_GYRO_MDPS_TO_DPS;
    float gy = snapshot.packet.gyro_mdps_y * AIRTRIXX_GYRO_MDPS_TO_DPS;
    float gz = snapshot.packet.gyro_mdps_z * AIRTRIXX_GYRO_MDPS_TO_DPS;
    float pitch = snapshot.packet.pitch_cdeg * AIRTRIXX_CDEG_TO_DEG;
    float roll = snapshot.packet.roll_cdeg * AIRTRIXX_CDEG_TO_DEG;

    Serial.print(",\"accel\":{\"x\":");
    Serial.print(ax, 3);
    Serial.print(",\"y\":");
    Serial.print(ay, 3);
    Serial.print(",\"z\":");
    Serial.print(az, 3);
    Serial.print("},\"gyro\":{\"x\":");
    Serial.print(gx, 3);
    Serial.print(",\"y\":");
    Serial.print(gy, 3);
    Serial.print(",\"z\":");
    Serial.print(gz, 3);
    Serial.print("},\"pitch\":");
    Serial.print(pitch, 2);
    Serial.print(",\"roll\":");
    Serial.print(roll, 2);
    Serial.print(",\"yaw\":null");
  } else {
    Serial.print(",\"accel\":{\"x\":null,\"y\":null,\"z\":null}");
    Serial.print(",\"gyro\":{\"x\":null,\"y\":null,\"z\":null}");
    Serial.print(",\"pitch\":null,\"roll\":null,\"yaw\":null");
  }
  Serial.print("}");
}

void printCamDockJson(const LatestCamDock &snapshot, uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  bool batteryValid = ok && snapshot.packet.battery_valid != 0;
  Serial.print("\"camdock\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"battery_level\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery_voltage\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery\":{\"status\":\"");
  if (!ok) {
    Serial.print("not_connected");
  } else if (!batteryValid) {
    Serial.print("invalid");
  } else {
    Serial.print("ok");
  }
  Serial.print("\",\"percent\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"voltage_v\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"adc_raw\":");
  if (ok) {
    Serial.print(snapshot.packet.battery_adc_raw);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"age_ms\":");
  if (ok) {
    Serial.print(nowMs - snapshot.received_ms);
  } else {
    Serial.print("null");
  }
  Serial.print("}");
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(snapshot.packet.header.t_ms);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"tof\":{\"left_mm\":");
  if (ok) {
    Serial.print(snapshot.packet.left_tof_mm);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"right_mm\":");
  if (ok) {
    Serial.print(snapshot.packet.right_tof_mm);
  } else {
    Serial.print("null");
  }
  Serial.print("},\"active_target\":\"");
  Serial.print(ok ? activePairToString(snapshot.packet.active_target) : "none");
  Serial.print("\"}");
}

void printNullableTempC(bool ok, uint8_t valid, int16_t centiC) {
  if (ok && valid != 0) {
    Serial.print(centiC / 100.0f, 2);
  } else {
    Serial.print("null");
  }
}

void printNullableDistanceMm(bool ok, uint8_t valid, uint16_t distanceMm) {
  if (ok && valid != 0) {
    Serial.print(distanceMm);
  } else {
    Serial.print("null");
  }
}

void printKeyboardJson(const LatestKeyboard &snapshot, uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  Serial.print("\"keyboard\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"input\":\"");
  Serial.print(ok ? "tof" : "off");
  Serial.print("\",\"battery_level\":null");
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(snapshot.packet.header.t_ms);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"tof\":{\"sensor_1_mm\":");
  printNullableDistanceMm(ok, snapshot.packet.valid_1, snapshot.packet.distance_mm_1);
  Serial.print(",\"sensor_2_mm\":");
  printNullableDistanceMm(ok, snapshot.packet.valid_2, snapshot.packet.distance_mm_2);
  Serial.print(",\"sensor_3_mm\":");
  printNullableDistanceMm(ok, snapshot.packet.valid_3, snapshot.packet.distance_mm_3);
  Serial.print(",\"sensor_4_mm\":");
  printNullableDistanceMm(ok, snapshot.packet.valid_4, snapshot.packet.distance_mm_4);
  Serial.print("},\"valid\":{\"sensor_1\":");
  Serial.print(ok && snapshot.packet.valid_1 != 0 ? "true" : "false");
  Serial.print(",\"sensor_2\":");
  Serial.print(ok && snapshot.packet.valid_2 != 0 ? "true" : "false");
  Serial.print(",\"sensor_3\":");
  Serial.print(ok && snapshot.packet.valid_3 != 0 ? "true" : "false");
  Serial.print(",\"sensor_4\":");
  Serial.print(ok && snapshot.packet.valid_4 != 0 ? "true" : "false");
  Serial.print("}}");
}

void printFansJson(const LatestFans &snapshot, uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  bool fanOn = ok && snapshot.packet.fan_on != 0;
  bool batteryValid = ok && snapshot.packet.battery_valid != 0;
  Serial.print("\"fans\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"input\":\"");
  Serial.print(fanOn ? "on" : "off");
  Serial.print("\",\"battery_level\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery_voltage\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery\":{\"status\":\"");
  if (!ok) {
    Serial.print("not_connected");
  } else if (!batteryValid) {
    Serial.print("invalid");
  } else {
    Serial.print("ok");
  }
  Serial.print("\",\"percent\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_percent);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"voltage_v\":");
  if (batteryValid) {
    Serial.print(snapshot.packet.battery_mv / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"adc_raw\":");
  if (ok) {
    Serial.print(snapshot.packet.battery_adc_raw);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"age_ms\":");
  if (ok) {
    Serial.print(nowMs - snapshot.received_ms);
  } else {
    Serial.print("null");
  }
  Serial.print("}");
  Serial.print(",\"fan_on\":");
  Serial.print(fanOn ? "true" : "false");
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(snapshot.packet.header.t_ms);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"last_command_sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.last_command_sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"temps\":{\"sensor_1_c\":");
  printNullableTempC(ok, snapshot.packet.temp1_valid, snapshot.packet.temp1_centi_c);
  Serial.print(",\"sensor_2_c\":");
  printNullableTempC(ok, snapshot.packet.temp2_valid, snapshot.packet.temp2_centi_c);
  Serial.print("}}");
}

const char *chargingDockChannelName(uint8_t index) {
  switch (index) {
    case 0: return "FN";
    case 1: return "AD";
    case 2: return "KB";
    case 3: return "WB";
    default: return "CH";
  }
}

bool maskHas(uint8_t mask, uint8_t index) {
  return (mask & (1 << index)) != 0;
}

const char *chargingDockInput(const ChargingDockStatusPacket &packet, bool ok) {
  if (!ok) {
    return "off";
  }
  if (packet.hot_mask != 0) {
    return "hot";
  }
  if (packet.charging_mask != 0) {
    return "charging";
  }
  return "idle";
}

const char *chargingDockChannelStatus(const ChargingDockStatusPacket &packet, bool ok, uint8_t index) {
  if (!ok) {
    return "not_connected";
  }
  if (!maskHas(packet.ina_valid_mask, index)) {
    return "ina_error";
  }
  if (maskHas(packet.hot_mask, index)) {
    return "hot";
  }
  if (!maskHas(packet.battery_present_mask, index)) {
    return "no_battery";
  }
  if (maskHas(packet.full_mask, index)) {
    return "full";
  }
  if (maskHas(packet.charging_mask, index)) {
    return "charging";
  }
  return "idle";
}

void printChargingDockJson(const LatestChargingDock &snapshot, uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  const ChargingDockStatusPacket &packet = snapshot.packet;
  uint16_t presentCount = 0;
  uint16_t chargingCount = 0;
  uint16_t percentSum = 0;
  uint32_t mvSum = 0;

  if (ok) {
    for (uint8_t i = 0; i < AIRTRIXX_CHARGING_DOCK_CHANNELS; ++i) {
      if (maskHas(packet.battery_present_mask, i)) {
        presentCount++;
        percentSum += packet.battery_percent[i];
        mvSum += packet.battery_mv[i];
      }
      if (maskHas(packet.charging_mask, i)) {
        chargingCount++;
      }
    }
  }

  Serial.print("\"charging_dock\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"input\":\"");
  Serial.print(chargingDockInput(packet, ok));
  Serial.print("\",\"battery_level\":");
  if (presentCount > 0) {
    Serial.print(static_cast<int>((percentSum + (presentCount / 2)) / presentCount));
  } else {
    Serial.print("null");
  }
  Serial.print(",\"battery_voltage\":");
  if (presentCount > 0) {
    Serial.print((static_cast<float>(mvSum) / presentCount) / 1000.0f, 3);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(packet.header.t_ms);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"active_tab\":");
  if (ok) {
    Serial.print(packet.active_tab);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"priority_channel\":");
  if (ok && packet.priority_channel >= 0 &&
      packet.priority_channel < static_cast<int8_t>(AIRTRIXX_CHARGING_DOCK_CHANNELS)) {
    Serial.print("\"");
    Serial.print(chargingDockChannelName(packet.priority_channel));
    Serial.print("\"");
  } else {
    Serial.print("null");
  }
  Serial.print(",\"present_count\":");
  Serial.print(ok ? presentCount : 0);
  Serial.print(",\"charging_count\":");
  Serial.print(ok ? chargingCount : 0);
  Serial.print(",\"channels\":[");
  for (uint8_t i = 0; i < AIRTRIXX_CHARGING_DOCK_CHANNELS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    bool inaValid = ok && maskHas(packet.ina_valid_mask, i);
    bool batteryPresent = ok && maskHas(packet.battery_present_mask, i);
    bool tempValid = ok && maskHas(packet.temp_valid_mask, i);
    Serial.print("{\"name\":\"");
    Serial.print(chargingDockChannelName(i));
    Serial.print("\",\"status\":\"");
    Serial.print(chargingDockChannelStatus(packet, ok, i));
    Serial.print("\",\"charging\":");
    Serial.print(ok && maskHas(packet.charging_mask, i) ? "true" : "false");
    Serial.print(",\"battery_level\":");
    if (batteryPresent) {
      Serial.print(packet.battery_percent[i]);
    } else {
      Serial.print("null");
    }
    Serial.print(",\"battery_voltage\":");
    if (batteryPresent) {
      Serial.print(packet.battery_mv[i] / 1000.0f, 3);
    } else {
      Serial.print("null");
    }
    Serial.print(",\"current_ma\":");
    if (inaValid) {
      Serial.print(packet.current_ma[i]);
    } else {
      Serial.print("null");
    }
    Serial.print(",\"temp_c\":");
    if (tempValid) {
      Serial.print(packet.temp_centi_c[i] / 100.0f, 2);
    } else {
      Serial.print("null");
    }
    Serial.print(",\"energy_mah\":");
    Serial.print(ok ? packet.energy_mah[i] : 0);
    Serial.print("}");
  }
  Serial.print("]}");
}

void printFutureDeviceJson(const char *name) {
  Serial.print("\"");
  Serial.print(name);
  Serial.print("\":{\"status\":\"TBD\",\"input\":\"TBD\",\"battery_level\":null}");
}

void printAudioDockJson(const LatestAudioDock &snapshot, uint32_t nowMs) {
  bool ok = snapshot.seen && (nowMs - snapshot.received_ms <= DEVICE_TIMEOUT_MS);
  Serial.print("\"audiodock\":{");
  Serial.print("\"status\":\"");
  Serial.print(ok ? "ok" : "not_connected");
  Serial.print("\",\"battery_level\":null");
  Serial.print(",\"sequence\":");
  if (ok) {
    Serial.print(snapshot.packet.header.sequence);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"t_ms\":");
  if (ok) {
    Serial.print(snapshot.packet.header.t_ms);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"clap_detected\":");
  Serial.print(ok && snapshot.packet.clap_detected ? "true" : "false");
  Serial.print(",\"clap_type\":");
  if (ok) {
    Serial.print(snapshot.packet.clap_type);
  } else {
    Serial.print("null");
  }
  Serial.print("}");
}

void printJsonState() {
  LatestWristband wristSnapshot;
  LatestBatteryStatus wristBatterySnapshot;
  LatestCamDock camSnapshot;
  LatestFans fansSnapshot;
  LatestKeyboard keyboardSnapshot;
  LatestChargingDock chargingDockSnapshot;
  LatestAudioDock audiodockSnapshot;
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&stateMux);
  wristSnapshot = latestWristband;
  wristBatterySnapshot = latestWristbandBattery;
  camSnapshot = latestCamDock;
  fansSnapshot = latestFans;
  keyboardSnapshot = latestKeyboard;
  chargingDockSnapshot = latestChargingDock;
  audiodockSnapshot = latestAudioDock;
  portEXIT_CRITICAL(&stateMux);

  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print("{\"t_ms\":");
    Serial.print(nowMs);
    Serial.print(",\"sequence\":");
    Serial.print(++antennaJsonSequence);
    
    uint8_t mac[6] = {};
    WiFi.macAddress(mac);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print(",\"antenna_mac\":\"");
    Serial.print(macStr);
    Serial.print("\"");

    Serial.print(",\"devices\":{");
    printWristbandJson(wristSnapshot, wristBatterySnapshot, nowMs);
    Serial.print(",");
    printCamDockJson(camSnapshot, nowMs);
    Serial.print(",");
    printKeyboardJson(keyboardSnapshot, nowMs);
    Serial.print(",");
    printChargingDockJson(chargingDockSnapshot, nowMs);
    Serial.print(",");
    printAudioDockJson(audiodockSnapshot, nowMs);
    Serial.print(",");
    printFansJson(fansSnapshot, nowMs);
    Serial.println("}}");
    xSemaphoreGive(serialMutex);
  }
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  Serial.begin(AIRTRIXX_SERIAL_BAUD);
  delay(200);

  // Initialize the thread-safe FreeRTOS Mutex for Serial operations
  serialMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS queue for Audio Dock chunks
  audioChunkQueue = xQueueCreate(64, sizeof(AudioDockChunkPacket));

  configureWiFiChannel();
  
  // Read and print actual physical MAC address
  uint8_t mac[6] = {};
  WiFi.macAddress(mac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print("ANTENNA_MAC: ");
    Serial.println(macStr);
    xSemaphoreGive(serialMutex);
  }

  if (esp_now_init() != ESP_OK) {
    debugPrintln("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  addEspNowPeer(CAMDOCK_MAC_PLACEHOLDER);
  addEspNowPeer(WRISTBAND_MAC_PLACEHOLDER);
  addEspNowPeer(KEYBOARD_MAC_PLACEHOLDER);
  addEspNowPeer(CHARGING_DOCK_MAC_PLACEHOLDER);
  addEspNowPeer(FANS_MAC_PLACEHOLDER);
  addEspNowPeer(AUDIODOCK_MAC_PLACEHOLDER);
  addEspNowPeer(ESPNOW_BROADCAST_MAC);
}

void pumpAudioDockChunks() {
  if (audioChunkQueue == NULL) return;

  AudioDockChunkPacket packet;
  while (xQueueReceive(audioChunkQueue, &packet, 0) == pdPASS) {
    char hexBuf[401];
    uint16_t writeLen = packet.chunk_len;
    if (writeLen > 200) writeLen = 200;
    for (uint16_t i = 0; i < writeLen; ++i) {
      sprintf(hexBuf + (i * 2), "%02X", packet.data[i]);
    }
    hexBuf[writeLen * 2] = '\0';
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.print("AUDIODOCK_AUDIO:");
      Serial.println(hexBuf);
      xSemaphoreGive(serialMutex);
    }
  }
}

void loop() {
  pumpAudioDockChunks();
  pumpSerialCommands();

  uint32_t nowMs = millis();
  
  if (isStreamingAudioDock && (nowMs - lastAudioDockChunkMs >= AUDIODOCK_STREAM_TIMEOUT_MS)) {
    isStreamingAudioDock = false;
  }

  const uint32_t intervalMs = 1000UL / ANTENNA_JSON_HZ;
  if (!isStreamingAudioDock && (nowMs - lastJsonMs >= intervalMs)) {
    lastJsonMs = nowMs;
    printJsonState();
  }
}
