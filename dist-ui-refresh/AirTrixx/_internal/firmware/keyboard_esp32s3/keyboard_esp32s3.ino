#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <VL53L0X.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

static const uint8_t SDA_PIN = 8;
static const uint8_t SCL_PIN = 9;
static const uint8_t TCA_ADDR = 0x70;
static const uint8_t VL53L0X_ADDR = 0x29;
static const uint8_t SENSOR_COUNT = 4;
static const uint8_t NO_CHANNEL = 0xFF;
static const uint32_t KEYBOARD_SERIAL_BAUD = 115200;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;
static const uint32_t SENSOR_RESCAN_INTERVAL_MS = 2000;
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.

static VL53L0X sensor0;
static VL53L0X sensor1;
static VL53L0X sensor2;
static VL53L0X sensor3;
static VL53L0X *tofSensors[SENSOR_COUNT] = {&sensor0, &sensor1, &sensor2, &sensor3};

static bool sensorReady[SENSOR_COUNT] = {};
static bool distanceValid[SENSOR_COUNT] = {};
static uint8_t sensorChannel[SENSOR_COUNT] = {NO_CHANNEL, NO_CHANNEL, NO_CHANNEL, NO_CHANNEL};
static uint16_t distanceMm[SENSOR_COUNT] = {};
static uint16_t keyboardSequence = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t lastSensorRescanMs = 0;
static uint32_t espNowSendOkCount = 0;
static uint32_t espNowSendFailCount = 0;
static uint32_t readOkCount[SENSOR_COUNT] = {};
static uint32_t readFailCount[SENSOR_COUNT] = {};

void statusPrintln(const String &message) {
  if (DEBUG_SERIAL) {
    Serial.println(message);
  } else {
    Serial.println(message);
  }
}

void printMacAddress(const uint8_t mac[6]) {
  for (int i = 0; i < 6; ++i) {
    if (i > 0) {
      Serial.print(":");
    }
    if (mac[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
}

bool tcaSelect(uint8_t channel) {
  if (channel > 7) {
    return false;
  }
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool i2cAddressPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint8_t readySensorCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (sensorReady[i]) {
      count++;
    }
  }
  return count;
}

bool channelAlreadyAssigned(uint8_t channel) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (sensorReady[i] && sensorChannel[i] == channel) {
      return true;
    }
  }
  return false;
}

void printI2CScanForSelectedBus(const String &label) {
  Serial.print("[KEYBOARD] I2C scan ");
  Serial.print(label);
  Serial.print(":");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x");
      if (address < 0x10) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.print(" none");
  }
  Serial.println();
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

  uint8_t mac[6] = {};
  WiFi.macAddress(mac);
  Serial.print("[KEYBOARD] WiFi STA MAC=");
  printMacAddress(mac);
  Serial.print(", channel=");
  Serial.println(ESPNOW_CHANNEL);
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
    statusPrintln("[KEYBOARD] ESP-NOW add antenna peer failed, err=" + String(result));
    return false;
  }
  return true;
}

bool initTofSensor(uint8_t index, uint8_t channel) {
  if (!tcaSelect(channel)) {
    return false;
  }
  delay(5);
  if (!i2cAddressPresent(VL53L0X_ADDR)) {
    return false;
  }
  VL53L0X *sensor = tofSensors[index];
  if (!sensor->init()) {
    return false;
  }
  sensor->setTimeout(80);
  sensor->setMeasurementTimingBudget(20000);
  sensorChannel[index] = channel;
  sensorReady[index] = true;
  return true;
}

void clearTofAssignments() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    sensorReady[i] = false;
    sensorChannel[i] = NO_CHANNEL;
    distanceValid[i] = false;
  }
}

void scanForTofSensors() {
  clearTofAssignments();
  Serial.println("[KEYBOARD] Scanning TCA channels for VL53L0X sensors...");
  printI2CScanForSelectedBus("root");

  uint8_t slot = 0;
  for (uint8_t channel = 0; channel < 8 && slot < SENSOR_COUNT; ++channel) {
    if (!tcaSelect(channel)) {
      Serial.print("[KEYBOARD] TCA channel ");
      Serial.print(channel);
      Serial.println(" select failed");
      continue;
    }
    delay(5);
    printI2CScanForSelectedBus("ch" + String(channel));

    if (!i2cAddressPresent(VL53L0X_ADDR)) {
      continue;
    }

    if (initTofSensor(slot, channel)) {
      Serial.print("[KEYBOARD] VL53L0X sensor ");
      Serial.print(slot + 1);
      Serial.print(" bound to TCA channel ");
      Serial.println(channel);
      slot++;
    } else {
      Serial.print("[KEYBOARD] VL53L0X init failed on TCA channel ");
      Serial.println(channel);
    }
  }

  Serial.print("[KEYBOARD] active sensor count=");
  Serial.println(readySensorCount());
}

void readTofSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    distanceValid[i] = false;
    if (!sensorReady[i] || sensorChannel[i] == NO_CHANNEL) {
      delay(2);
      continue;
    }

    if (!tcaSelect(sensorChannel[i])) {
      sensorReady[i] = false;
      readFailCount[i]++;
      continue;
    }

    uint16_t value = tofSensors[i]->readRangeSingleMillimeters();
    if (tofSensors[i]->timeoutOccurred() || value == 0 || value >= 8190) {
      readFailCount[i]++;
      distanceMm[i] = value;
      continue;
    }

    distanceMm[i] = value;
    distanceValid[i] = true;
    readOkCount[i]++;
  }
}

void sendKeyboardTof() {
  KeyboardTofPacket packet = {};
  fillHeader(packet.header,
             MSG_KEYBOARD_TOF,
             DEVICE_KEYBOARD,
             ++keyboardSequence,
             millis(),
             false);
  packet.distance_mm_1 = distanceMm[0];
  packet.distance_mm_2 = distanceMm[1];
  packet.distance_mm_3 = distanceMm[2];
  packet.distance_mm_4 = distanceMm[3];
  packet.valid_1 = distanceValid[0] ? 1 : 0;
  packet.valid_2 = distanceValid[1] ? 1 : 0;
  packet.valid_3 = distanceValid[2] ? 1 : 0;
  packet.valid_4 = distanceValid[3] ? 1 : 0;

  esp_err_t result = esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                                  reinterpret_cast<uint8_t *>(&packet),
                                  sizeof(packet));
  if (result == ESP_OK) {
    espNowSendOkCount++;
  } else {
    espNowSendFailCount++;
  }
}

void printStatus() {
  Serial.print("[KEYBOARD] status seq=");
  Serial.print(keyboardSequence);
  Serial.print(", distances=");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i > 0) {
      Serial.print("/");
    }
    if (distanceValid[i]) {
      Serial.print(distanceMm[i]);
    } else {
      Serial.print("invalid");
    }
  }
  Serial.print(", ready=");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(sensorReady[i] ? "1" : "0");
  }
  Serial.print(", channels=");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i > 0) {
      Serial.print("/");
    }
    if (sensorChannel[i] == NO_CHANNEL) {
      Serial.print("-");
    } else {
      Serial.print(sensorChannel[i]);
    }
  }
  Serial.print(", read_ok=");
  Serial.print(readOkCount[0]);
  Serial.print("/");
  Serial.print(readOkCount[1]);
  Serial.print("/");
  Serial.print(readOkCount[2]);
  Serial.print("/");
  Serial.print(readOkCount[3]);
  Serial.print(", read_fail=");
  Serial.print(readFailCount[0]);
  Serial.print("/");
  Serial.print(readFailCount[1]);
  Serial.print("/");
  Serial.print(readFailCount[2]);
  Serial.print("/");
  Serial.print(readFailCount[3]);
  Serial.print(", send_ok/fail=");
  Serial.print(espNowSendOkCount);
  Serial.print("/");
  Serial.println(espNowSendFailCount);
}

void setup() {
  Serial.begin(KEYBOARD_SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("[KEYBOARD] AirTrixx Keyboard ToF ESP32-S3 booting...");
  Serial.println("[KEYBOARD] I2C SDA=" + String(SDA_PIN) +
                 " SCL=" + String(SCL_PIN) + " TCA=0x70");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.print("[KEYBOARD] TCA ");
  Serial.println(i2cAddressPresent(TCA_ADDR) ? "found" : "not found");
  scanForTofSensors();

  configureWiFiChannel();
  if (esp_now_init() != ESP_OK) {
    statusPrintln("[KEYBOARD] ESP-NOW init failed.");
    return;
  }
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
  Serial.println("[KEYBOARD] Setup complete.");
}

void loop() {
  readTofSensors();

  uint32_t nowMs = millis();
  if (readySensorCount() < SENSOR_COUNT &&
      nowMs - lastSensorRescanMs >= SENSOR_RESCAN_INTERVAL_MS) {
    lastSensorRescanMs = nowMs;
    scanForTofSensors();
  }

  const uint32_t reportIntervalMs = 1000UL / KEYBOARD_REPORT_HZ;
  if (nowMs - lastReportMs >= reportIntervalMs) {
    lastReportMs = nowMs;
    sendKeyboardTof();
  }

  if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintMs = nowMs;
    printStatus();
  }

  delay(5);
}
