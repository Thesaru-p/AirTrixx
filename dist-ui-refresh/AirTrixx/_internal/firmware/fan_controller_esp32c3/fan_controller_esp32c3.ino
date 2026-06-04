#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

// Same wiring as the proven standalone fan sketch.
static const uint8_t FAN_CONTROL_PIN = 6;
static const uint8_t ONE_WIRE_BUS = 0;
static const uint8_t BATTERY_ADC_PIN = 1;
static const bool FAN_ACTIVE_HIGH = true;

static const uint32_t FANS_SERIAL_BAUD = 115200;
static const uint32_t TEMP_READ_INTERVAL_MS = 1000;
static const uint32_t BATTERY_READ_INTERVAL_MS = 1000;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;
static const bool STATUS_SERIAL = true;
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.
static const float BATTERY_DIVIDER_RATIO = 147.0f / 47.0f;
static const float BATTERY_VALID_MIN_V = 2.0f;
static const float BATTERY_LOW_THRESHOLD_V = 3.5f;
static const uint8_t BATTERY_ADC_SAMPLES = 8;

static OneWire oneWire(ONE_WIRE_BUS);
static DallasTemperature sensors(&oneWire);

static uint16_t fanSequence = 0;
static uint16_t lastFanCommandSequence = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastTempReadMs = 0;
static uint32_t lastBatteryReadMs = 0;
static uint32_t lastTempSearchMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t espNowSendOkCount = 0;
static uint32_t espNowSendFailCount = 0;
static uint32_t fanCommandRxCount = 0;
static uint32_t badPacketCount = 0;
static uint32_t otaStartRxCount = 0;
static uint32_t otaChunkRxCount = 0;
static uint32_t otaFailCount = 0;
static uint32_t lastSendOkMs = 0;
static uint32_t lastSendFailMs = 0;

static bool fanOn = false;
static bool temp1Valid = false;
static bool temp2Valid = false;
static bool batteryValid = false;
static bool batteryLow = false;
static float temp1C = NAN;
static float temp2C = NAN;
static float batteryVoltage = 0.0f;
static uint8_t batteryPercent = 0;
static uint16_t batteryAdcRaw = 0;
static uint8_t tempSensorCount = 0;

static FanCommandPacket pendingCommand = {};
static bool hasPendingCommand = false;
static portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
static OtaStartPacket pendingOtaPacket = {};
static bool hasPendingOta = false;
static bool otaInProgress = false;
static portMUX_TYPE otaMux = portMUX_INITIALIZER_UNLOCKED;
static OtaStartPacket assembledOtaPacket = {};
static bool otaSsidComplete = false;
static bool otaPasswordComplete = false;
static bool otaUrlComplete = false;
static bool otaMd5Complete = false;

void statusPrintln(const String &message) {
  if (DEBUG_SERIAL || STATUS_SERIAL) {
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

void setFan(bool enabled) {
  fanOn = enabled;
  bool outputHigh = FAN_ACTIVE_HIGH ? enabled : !enabled;
  digitalWrite(FAN_CONTROL_PIN, outputHigh ? HIGH : LOW);
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

  if (STATUS_SERIAL) {
    uint8_t mac[6] = {};
    WiFi.macAddress(mac);
    Serial.print("[FANS] WiFi STA MAC=");
    printMacAddress(mac);
    Serial.print(", channel=");
    Serial.println(ESPNOW_CHANNEL);
    Serial.print("[FANS] WiFi tx_power=8.5dBm, espnow_rate=1Mbps long preamble");
    Serial.println();
  }
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
    statusPrintln("[FANS] ESP-NOW add antenna peer failed, err=" + String(result));
    return false;
  }
  return true;
}

bool readTempByIndex(uint8_t index, float &outC) {
  float value = sensors.getTempCByIndex(index);
  if (value == DEVICE_DISCONNECTED_C || value < -40.0f || value > 125.0f) {
    outC = NAN;
    return false;
  }
  outC = value;
  return true;
}

void refreshTemperatureDeviceCounts() {
  sensors.begin();
  sensors.setResolution(10);
  tempSensorCount = sensors.getDeviceCount();
}

void updateTemperatures() {
  uint32_t nowMs = millis();
  if (tempSensorCount < 2) {
    if (lastTempSearchMs == 0 || nowMs - lastTempSearchMs >= 2000) {
      lastTempSearchMs = nowMs;
      refreshTemperatureDeviceCounts();
    }
  }

  sensors.requestTemperatures();

  float value = NAN;
  temp1Valid = readTempByIndex(0, value);
  temp1C = value;
  temp2Valid = readTempByIndex(1, value);
  temp2C = value;
}

int16_t tempToCentiC(float valueC, bool valid) {
  if (!valid) {
    return 0;
  }
  float bounded = constrain(valueC, -40.0f, 125.0f);
  return static_cast<int16_t>(lroundf(bounded * 100.0f));
}

uint8_t batteryPercentFromVoltage(float voltage) {
  if (voltage <= 3.20f) {
    return 0;
  }
  if (voltage >= 4.20f) {
    return 100;
  }

  struct VoltagePoint {
    float voltage;
    uint8_t percent;
  };
  static const VoltagePoint curve[] = {
    {3.20f, 0},
    {3.50f, 10},
    {3.70f, 35},
    {3.85f, 55},
    {3.95f, 70},
    {4.05f, 85},
    {4.20f, 100},
  };

  for (size_t i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
    if (voltage <= curve[i].voltage) {
      const VoltagePoint &lower = curve[i - 1];
      const VoltagePoint &upper = curve[i];
      float span = upper.voltage - lower.voltage;
      float fraction = span > 0.0f ? (voltage - lower.voltage) / span : 0.0f;
      return static_cast<uint8_t>(lroundf(lower.percent +
                                          fraction * (upper.percent - lower.percent)));
    }
  }
  return 100;
}

void updateBatteryStatus() {
  uint32_t nowMs = millis();
  if (nowMs - lastBatteryReadMs < BATTERY_READ_INTERVAL_MS) {
    return;
  }
  lastBatteryReadMs = nowMs;

  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
    rawSum += analogRead(BATTERY_ADC_PIN);
    mvSum += analogReadMilliVolts(BATTERY_ADC_PIN);
    delayMicroseconds(200);
  }

  batteryAdcRaw = rawSum / BATTERY_ADC_SAMPLES;
  float pinVoltage = (static_cast<float>(mvSum) / BATTERY_ADC_SAMPLES) / 1000.0f;
  batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;
  batteryValid = batteryVoltage >= BATTERY_VALID_MIN_V;
  batteryPercent = batteryValid ? batteryPercentFromVoltage(batteryVoltage) : 0;
  batteryLow = batteryValid && batteryVoltage < BATTERY_LOW_THRESHOLD_V;
}

void sendFanStatus() {
  FanStatusPacket packet = {};
  fillHeader(packet.header,
             MSG_FAN_STATUS,
             DEVICE_FANS,
             ++fanSequence,
             millis(),
             batteryLow);
  packet.fan_on = fanOn ? 1 : 0;
  packet.temp1_valid = temp1Valid ? 1 : 0;
  packet.temp2_valid = temp2Valid ? 1 : 0;
  packet.temp1_centi_c = tempToCentiC(temp1C, temp1Valid);
  packet.temp2_centi_c = tempToCentiC(temp2C, temp2Valid);
  packet.last_command_sequence = lastFanCommandSequence;
  packet.battery_mv = batteryValid ? static_cast<uint16_t>(lroundf(batteryVoltage * 1000.0f)) : 0;
  packet.battery_adc_raw = batteryAdcRaw;
  packet.battery_percent = batteryValid ? batteryPercent : 0;
  packet.battery_valid = batteryValid ? 1 : 0;

  esp_err_t result = esp_now_send(ESPNOW_BROADCAST_MAC,
                                  reinterpret_cast<uint8_t *>(&packet),
                                  sizeof(packet));
  if (result != ESP_OK) {
    espNowSendFailCount++;
    lastSendFailMs = millis();
    statusPrintln("[FANS] ESP-NOW fan status send failed, err=" + String(result));
  } else {
    espNowSendOkCount++;
    lastSendOkMs = millis();
  }
}

void applyFanCommand(const FanCommandPacket &command) {
  lastFanCommandSequence = command.header.sequence;
  setFan(command.fan_on != 0);
  statusPrintln("[FANS] command seq=" + String(lastFanCommandSequence) +
                ", fan=" + String(fanOn ? "on" : "off"));
  sendFanStatus();
}

void resetOtaAssembly() {
  assembledOtaPacket = {};
  otaSsidComplete = false;
  otaPasswordComplete = false;
  otaUrlComplete = false;
  otaMd5Complete = false;
}

bool otaFieldBuffer(uint8_t fieldId, char *&buffer, size_t &capacity, bool *&completeFlag) {
  switch (fieldId) {
    case OTA_FIELD_SSID:
      buffer = assembledOtaPacket.ssid;
      capacity = sizeof(assembledOtaPacket.ssid);
      completeFlag = &otaSsidComplete;
      return true;
    case OTA_FIELD_PASSWORD:
      buffer = assembledOtaPacket.password;
      capacity = sizeof(assembledOtaPacket.password);
      completeFlag = &otaPasswordComplete;
      return true;
    case OTA_FIELD_URL:
      buffer = assembledOtaPacket.url;
      capacity = sizeof(assembledOtaPacket.url);
      completeFlag = &otaUrlComplete;
      return true;
    case OTA_FIELD_MD5:
      buffer = assembledOtaPacket.md5;
      capacity = sizeof(assembledOtaPacket.md5);
      completeFlag = &otaMd5Complete;
      return true;
    default:
      return false;
  }
}

void commitAssembledOta(const AirTrixxPacketHeader &header) {
  if (!otaSsidComplete || !otaUrlComplete ||
      assembledOtaPacket.ssid[0] == '\0' || assembledOtaPacket.url[0] == '\0') {
    badPacketCount++;
    return;
  }

  assembledOtaPacket.header = header;
  assembledOtaPacket.header.msg_type = MSG_OTA_START;
  portENTER_CRITICAL(&otaMux);
  pendingOtaPacket = assembledOtaPacket;
  hasPendingOta = true;
  otaStartRxCount++;
  portEXIT_CRITICAL(&otaMux);
}

void handleOtaChunkPacket(const OtaChunkPacket &chunk) {
  otaChunkRxCount++;

  if (chunk.field_id == OTA_FIELD_RESET) {
    resetOtaAssembly();
    return;
  }

  if (chunk.field_id == OTA_FIELD_COMMIT) {
    commitAssembledOta(chunk.header);
    return;
  }

  if (chunk.chunk_len > AIRTRIXX_OTA_CHUNK_BYTES) {
    badPacketCount++;
    return;
  }

  char *buffer = nullptr;
  size_t capacity = 0;
  bool *completeFlag = nullptr;
  if (!otaFieldBuffer(chunk.field_id, buffer, capacity, completeFlag)) {
    badPacketCount++;
    return;
  }

  if (chunk.total_len >= capacity ||
      chunk.offset > chunk.total_len ||
      static_cast<size_t>(chunk.offset) + chunk.chunk_len > chunk.total_len) {
    badPacketCount++;
    return;
  }

  if (chunk.offset == 0) {
    memset(buffer, 0, capacity);
    *completeFlag = false;
  }
  memcpy(buffer + chunk.offset, chunk.data, chunk.chunk_len);
  buffer[chunk.total_len] = '\0';
  if (static_cast<size_t>(chunk.offset) + chunk.chunk_len >= chunk.total_len) {
    *completeFlag = true;
  }
}

void handleIncomingPacket(const uint8_t *data, int len) {
  if (len < static_cast<int>(sizeof(AirTrixxPacketHeader))) {
    badPacketCount++;
    return;
  }

  AirTrixxPacketHeader header = {};
  memcpy(&header, data, sizeof(header));
  if (header.protocol_version != AIRTRIXX_PROTOCOL_VERSION ||
      header.device_id != DEVICE_ANTENNA) {
    badPacketCount++;
    return;
  }

  if (header.msg_type == MSG_FAN_COMMAND && len == static_cast<int>(sizeof(FanCommandPacket))) {
    FanCommandPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    portENTER_CRITICAL(&commandMux);
    pendingCommand = packet;
    hasPendingCommand = true;
    fanCommandRxCount++;
    portEXIT_CRITICAL(&commandMux);
    return;
  }

  if (header.msg_type == MSG_OTA_START && len == static_cast<int>(sizeof(OtaStartPacket))) {
    OtaStartPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    packet.ssid[AIRTRIXX_OTA_SSID_MAX] = '\0';
    packet.password[AIRTRIXX_OTA_PASSWORD_MAX] = '\0';
    packet.url[AIRTRIXX_OTA_URL_MAX] = '\0';
    packet.md5[AIRTRIXX_OTA_MD5_MAX] = '\0';
    portENTER_CRITICAL(&otaMux);
    pendingOtaPacket = packet;
    hasPendingOta = true;
    otaStartRxCount++;
    portEXIT_CRITICAL(&otaMux);
    return;
  }

  if (header.msg_type == MSG_OTA_CHUNK && len == static_cast<int>(sizeof(OtaChunkPacket))) {
    OtaChunkPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    handleOtaChunkPacket(packet);
    return;
  }

  badPacketCount++;
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

bool initializeEspNowStack() {
  if (esp_now_init() != ESP_OK) {
    statusPrintln("[FANS] ESP-NOW init failed.");
    return false;
  }
  esp_now_register_recv_cb(onDataRecv);
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
  addEspNowPeer(ESPNOW_BROADCAST_MAC);
  return true;
}

void restoreEspNowAfterOtaFailure() {
  WiFi.disconnect(true);
  delay(100);
  configureWiFiChannel();
  initializeEspNowStack();
}

void performOtaUpdate(const OtaStartPacket &packet) {
  otaInProgress = true;
  setFan(false);
  statusPrintln("[FANS] OTA requested, url=" + String(packet.url));

  esp_now_deinit();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);
  WiFi.begin(packet.ssid, packet.password);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 60000UL) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    otaFailCount++;
    statusPrintln("[FANS] OTA WiFi connect failed.");
    restoreEspNowAfterOtaFailure();
    otaInProgress = false;
    return;
  }

  statusPrintln("[FANS] OTA WiFi connected, IP=" + WiFi.localIP().toString());
  WiFiClient client;
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.onProgress([](int current, int total) {
    static uint32_t lastProgressMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastProgressMs >= 500) {
      lastProgressMs = nowMs;
      if (total > 0) {
        statusPrintln("[FANS] OTA progress " + String((current * 100) / total) + "%");
      }
    }
  });

  t_httpUpdate_return result = httpUpdate.update(client, String(packet.url));
  if (result == HTTP_UPDATE_OK) {
    statusPrintln("[FANS] OTA update ok; rebooting.");
    delay(100);
    ESP.restart();
  }

  otaFailCount++;
  statusPrintln("[FANS] OTA update failed, err=" +
                String(httpUpdate.getLastError()) + " " +
                httpUpdate.getLastErrorString());
  restoreEspNowAfterOtaFailure();
  otaInProgress = false;
}

void processPendingOta() {
  if (otaInProgress) {
    return;
  }

  OtaStartPacket packet = {};
  bool shouldStart = false;
  portENTER_CRITICAL(&otaMux);
  if (hasPendingOta) {
    packet = pendingOtaPacket;
    hasPendingOta = false;
    shouldStart = true;
  }
  portEXIT_CRITICAL(&otaMux);

  if (shouldStart) {
    performOtaUpdate(packet);
  }
}

void printConnectionStatus() {
  if (!STATUS_SERIAL) {
    return;
  }
  uint32_t nowMs = millis();
  Serial.print("[FANS] status t=");
  Serial.print(nowMs);
  Serial.print("ms, peer=");
  Serial.print(esp_now_is_peer_exist(ANTENNA_MAC_PLACEHOLDER) ? "yes" : "no");
  Serial.print(", fan=");
  Serial.print(fanOn ? "on" : "off");
  Serial.print(", temp1=");
  if (temp1Valid) {
    Serial.print(temp1C, 2);
  } else {
    Serial.print("invalid");
  }
  Serial.print(", temp2=");
  if (temp2Valid) {
    Serial.print(temp2C, 2);
  } else {
    Serial.print("invalid");
  }
  Serial.print(", temp_count=");
  Serial.print(tempSensorCount);
  Serial.print(", battery_v=");
  Serial.print(batteryVoltage, 3);
  Serial.print(", battery_pct=");
  Serial.print(batteryValid ? static_cast<int>(batteryPercent) : -1);
  Serial.print(", battery_adc=");
  Serial.print(batteryAdcRaw);
  Serial.print(", rx=");
  Serial.print(fanCommandRxCount);
  Serial.print(", bad=");
  Serial.print(badPacketCount);
  Serial.print(", send_ok=");
  Serial.print(espNowSendOkCount);
  Serial.print(", send_fail=");
  Serial.print(espNowSendFailCount);
  Serial.print(", ota_rx/fail=");
  Serial.print(otaStartRxCount);
  Serial.print("/");
  Serial.print(otaFailCount);
  Serial.print(", ota_chunks=");
  Serial.print(otaChunkRxCount);
  Serial.print(", last_send_ok_age_ms=");
  Serial.print(lastSendOkMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendOkMs));
  Serial.print(", last_send_fail_age_ms=");
  Serial.println(lastSendFailMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendFailMs));
}

void setup() {
  Serial.begin(FANS_SERIAL_BAUD);
  delay(1000);
  statusPrintln("");
  statusPrintln("[FANS] AirTrixx Fan Controller ESP32-C3 booting...");
  statusPrintln("[FANS] Fan pin GPIO" + String(FAN_CONTROL_PIN) +
                ", DS18B20 bus GPIO" + String(ONE_WIRE_BUS) +
                ", battery ADC GPIO" + String(BATTERY_ADC_PIN));

  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  refreshTemperatureDeviceCounts();
  for (uint8_t attempt = 1; attempt <= 10 && tempSensorCount < 2; ++attempt) {
    statusPrintln("[FANS] DS18B20 search attempt " + String(attempt) +
                  ", count=" + String(tempSensorCount));
    delay(250);
    refreshTemperatureDeviceCounts();
  }
  statusPrintln("[FANS] DS18B20 count before fan/WiFi init=" + String(tempSensorCount));

  pinMode(FAN_CONTROL_PIN, OUTPUT);
  setFan(false);

  updateTemperatures();
  lastBatteryReadMs = millis() - BATTERY_READ_INTERVAL_MS;
  updateBatteryStatus();

  configureWiFiChannel();
  if (!initializeEspNowStack()) {
    return;
  }
  statusPrintln("[FANS] Setup complete.");
}

void loop() {
  processPendingOta();
  if (otaInProgress) {
    return;
  }

  bool shouldApply = false;
  FanCommandPacket command = {};
  portENTER_CRITICAL(&commandMux);
  if (hasPendingCommand) {
    command = pendingCommand;
    hasPendingCommand = false;
    shouldApply = true;
  }
  portEXIT_CRITICAL(&commandMux);

  if (shouldApply) {
    applyFanCommand(command);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastTempReadMs >= TEMP_READ_INTERVAL_MS) {
    lastTempReadMs = nowMs;
    updateTemperatures();
  }
  updateBatteryStatus();

  const uint32_t reportIntervalMs = 1000UL / FANS_REPORT_HZ;
  if (nowMs - lastReportMs >= reportIntervalMs) {
    lastReportMs = nowMs;
    sendFanStatus();
  }

  if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintMs = nowMs;
    printConnectionStatus();
  }
}
