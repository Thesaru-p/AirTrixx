#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

#define SDA_PIN 8
#define SCL_PIN 9
#define STATUS_LED 4
#define BATTERY_ADC_PIN 1

#define MPU6050_ADDR_PRIMARY 0x68
#define MPU6050_ADDR_SECONDARY 0x69

// Set this to false if your ESP32-C3 SuperMini onboard LED works opposite.
static const bool STATUS_LED_ACTIVE_HIGH = true;
static const float BATTERY_DIVIDER_RATIO = 2.0f;
static const float BATTERY_EMPTY_V = 3.30f;
static const float BATTERY_FULL_V = 4.20f;
static const float BATTERY_LOW_THRESHOLD_V = 3.5f;
static const float BATTERY_LOW_CLEAR_THRESHOLD_V = 3.6f;
static const float BATTERY_SENSE_VALID_MIN_V = 2.0f;
static const uint32_t BATTERY_READ_INTERVAL_MS = 500;
static const uint32_t BATTERY_REPORT_AFTER_CONNECT_MS = 5000;
static const uint32_t BATTERY_REPORT_INTERVAL_MS = 5UL * 60UL * 1000UL;
static const uint32_t STATUS_LED_STARTUP_PULSE_MS = 120;
static const uint32_t STATUS_LED_LOW_BATTERY_INTERVAL_MS = 250;
static const uint32_t STATUS_LED_WAITING_PERIOD_MS = 1000;
static const uint32_t STATUS_LED_WAITING_ON_MS = 80;
static const uint32_t STATUS_LED_SENSOR_FAULT_PERIOD_MS = 2000;
static const uint32_t STATUS_LED_SENSOR_FAULT_PULSE_MS = 120;
static const uint32_t STATUS_LED_SENSOR_FAULT_GAP_MS = 120;
static const uint8_t BATTERY_ADC_SAMPLES = 8;

static bool imuReady = false;
static uint8_t mpu6050Addr = MPU6050_ADDR_PRIMARY;
static uint16_t wristSequence = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t lastBatteryReadMs = 0;
static uint32_t lastBatteryReportMs = 0;
static uint32_t lastLedBlinkMs = 0;
static uint32_t espNowSendOkCount = 0;
static uint32_t espNowSendFailCount = 0;
static volatile uint32_t espNowDeliveryOkCount = 0;
static volatile uint32_t espNowDeliveryFailCount = 0;
static uint32_t lastSendOkMs = 0;
static uint32_t lastSendFailMs = 0;
static volatile uint32_t lastDeliveryOkMs = 0;
static volatile uint32_t lastDeliveryFailMs = 0;
static bool antennaConnected = false;
static bool batteryLow = false;
static bool batterySenseValid = false;
static bool batteryReportScheduled = false;
static bool ledBlinkOn = false;
static uint32_t nextBatteryReportMs = 0;
static uint16_t batteryAdcRaw = 0;
static float batteryVoltage = 0.0f;
static uint8_t batteryPercent = 0;
static uint32_t imuUpdateOkCount = 0;
static uint32_t imuUpdateMissCount = 0;
static uint32_t otaStartRxCount = 0;
static uint32_t otaFailCount = 0;

static int16_t AccX = 0;
static int16_t AccY = 0;
static int16_t AccZ = 0;
static int16_t GyroX = 0;
static int16_t GyroY = 0;
static int16_t GyroZ = 0;

static float accelOffsetX = 0.0f;
static float accelOffsetY = 0.0f;
static float accelOffsetZ = 0.0f;
static float gyroOffsetX = 0.0f;
static float gyroOffsetY = 0.0f;
static float gyroOffsetZ = 0.0f;

static float accelX = 0.0f;
static float accelY = 0.0f;
static float accelZ = 1.0f;
static float gyroX = 0.0f;
static float gyroY = 0.0f;
static float gyroZ = 0.0f;
static float pitchDeg = 0.0f;
static float rollDeg = 0.0f;

static OtaStartPacket pendingOtaPacket = {};
static bool hasPendingOta = false;
static bool otaInProgress = false;
static portMUX_TYPE otaMux = portMUX_INITIALIZER_UNLOCKED;

// Wristband is not the USB JSON source, so local status logs are safe here.
static const bool STATUS_SERIAL = true;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;
static const float MPU6050_ACCEL_LSB_PER_G = 16384.0f;
static const float MPU6050_GYRO_LSB_PER_DPS = 131.0f;

void debugPrintln(const String &message) {
  if (DEBUG_SERIAL || STATUS_SERIAL) {
    Serial.println(message);
  }
}

void statusPrintln(const String &message) {
  if (STATUS_SERIAL) {
    Serial.println(message);
  }
}

void setStatusLed(bool on) {
  digitalWrite(STATUS_LED, on == STATUS_LED_ACTIVE_HIGH ? HIGH : LOW);
}

void runStatusLedSelfTest() {
  for (uint8_t i = 0; i < 3; ++i) {
    setStatusLed(true);
    delay(STATUS_LED_STARTUP_PULSE_MS);
    setStatusLed(false);
    delay(STATUS_LED_STARTUP_PULSE_MS);
  }
}

bool periodicPulseOn(uint32_t nowMs, uint32_t periodMs, uint32_t onMs) {
  return (nowMs % periodMs) < onMs;
}

bool repeatedPulseOn(uint32_t nowMs,
                     uint32_t periodMs,
                     uint8_t pulseCount,
                     uint32_t pulseMs,
                     uint32_t gapMs) {
  uint32_t phaseMs = nowMs % periodMs;
  for (uint8_t i = 0; i < pulseCount; ++i) {
    uint32_t pulseStartMs = i * (pulseMs + gapMs);
    if (phaseMs >= pulseStartMs && phaseMs < pulseStartMs + pulseMs) {
      return true;
    }
  }
  return false;
}

void refreshAntennaConnectionStatus(uint32_t nowMs) {
  bool connected = lastDeliveryOkMs != 0 &&
                   nowMs - lastDeliveryOkMs <= DEVICE_TIMEOUT_MS;
  if (connected == antennaConnected) {
    return;
  }

  antennaConnected = connected;
  if (antennaConnected) {
    batteryReportScheduled = true;
    nextBatteryReportMs = nowMs + BATTERY_REPORT_AFTER_CONNECT_MS;
    return;
  }

  batteryReportScheduled = false;
  nextBatteryReportMs = 0;
  ledBlinkOn = false;
  setStatusLed(false);
}

uint8_t batteryPercentFromVoltage(float voltage) {
  if (voltage <= BATTERY_EMPTY_V) {
    return 0;
  }
  if (voltage >= BATTERY_FULL_V) {
    return 100;
  }
  return static_cast<uint8_t>(lroundf((voltage - BATTERY_EMPTY_V) * 100.0f /
                                      (BATTERY_FULL_V - BATTERY_EMPTY_V)));
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
  batterySenseValid = batteryVoltage >= BATTERY_SENSE_VALID_MIN_V;
  batteryPercent = batterySenseValid ? batteryPercentFromVoltage(batteryVoltage) : 0;

  if (!batterySenseValid) {
    batteryLow = false;
    return;
  }

  if (batteryLow) {
    batteryLow = batteryVoltage < BATTERY_LOW_CLEAR_THRESHOLD_V;
  } else {
    batteryLow = batteryVoltage < BATTERY_LOW_THRESHOLD_V;
  }
}

void updateStatusLed() {
  uint32_t nowMs = millis();
  refreshAntennaConnectionStatus(nowMs);

  if (batterySenseValid && batteryLow) {
    if (nowMs - lastLedBlinkMs >= STATUS_LED_LOW_BATTERY_INTERVAL_MS) {
      lastLedBlinkMs = nowMs;
      ledBlinkOn = !ledBlinkOn;
      setStatusLed(ledBlinkOn);
    }
    return;
  }

  if (!imuReady) {
    setStatusLed(repeatedPulseOn(nowMs,
                                 STATUS_LED_SENSOR_FAULT_PERIOD_MS,
                                 3,
                                 STATUS_LED_SENSOR_FAULT_PULSE_MS,
                                 STATUS_LED_SENSOR_FAULT_GAP_MS));
    return;
  }

  if (!antennaConnected) {
    setStatusLed(periodicPulseOn(nowMs,
                                 STATUS_LED_WAITING_PERIOD_MS,
                                 STATUS_LED_WAITING_ON_MS));
    return;
  }

  ledBlinkOn = true;
  setStatusLed(true);
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

void printWiFiStatus() {
  if (!STATUS_SERIAL) {
    return;
  }

  uint8_t mac[6] = {};
  WiFi.macAddress(mac);
  uint8_t primaryChannel = 0;
  wifi_second_chan_t secondChannel = WIFI_SECOND_CHAN_NONE;
  esp_err_t channelResult = esp_wifi_get_channel(&primaryChannel, &secondChannel);

  Serial.print("[WRISTBAND] WiFi mode STA, MAC=");
  printMacAddress(mac);
  Serial.print(", configured_channel=");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(", active_channel=");
  if (channelResult == ESP_OK) {
    Serial.print(primaryChannel);
  } else {
    Serial.print("unknown err=");
    Serial.print(channelResult);
  }
  Serial.print(", power_save=off");
  Serial.println();
}

void configureWiFiChannel() {
  statusPrintln("[WRISTBAND] Turning WiFi on in STA mode...");
  
  // CRITICAL FIXES FOR DEFECTIVE SUPERMINI ANTENNA MATCHING
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Force RF synthesizer to stay fully awake & stable
  WiFi.disconnect();
  
  // Drop transmission power limit down to safe threshold to avoid self-blinding reflections (VSWR)
  WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  printWiFiStatus();
}

bool addEspNowPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    statusPrintln("[WRISTBAND] ESP-NOW antenna peer already exists.");
    return true;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    statusPrintln("[WRISTBAND] ESP-NOW add antenna peer failed, err=" + String(result));
    return false;
  }
  statusPrintln("[WRISTBAND] ESP-NOW antenna peer added.");
  return true;
}

void handleIncomingPacket(const uint8_t *data, int len) {
  if (len != static_cast<int>(sizeof(OtaStartPacket))) {
    return;
  }

  OtaStartPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (packet.header.protocol_version != AIRTRIXX_PROTOCOL_VERSION ||
      packet.header.msg_type != MSG_OTA_START ||
      packet.header.device_id != DEVICE_ANTENNA) {
    return;
  }

  packet.ssid[AIRTRIXX_OTA_SSID_MAX] = '\0';
  packet.password[AIRTRIXX_OTA_PASSWORD_MAX] = '\0';
  packet.url[AIRTRIXX_OTA_URL_MAX] = '\0';
  packet.md5[AIRTRIXX_OTA_MD5_MAX] = '\0';

  portENTER_CRITICAL(&otaMux);
  pendingOtaPacket = packet;
  hasPendingOta = true;
  otaStartRxCount++;
  portEXIT_CRITICAL(&otaMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info;
  handleIncomingPacket(incomingData, len);
}
#else
void onEspNowDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  (void)mac;
  handleIncomingPacket(incomingData, len);
}
#endif

void handleEspNowSendResult(esp_now_send_status_t status) {
  uint32_t nowMs = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    espNowDeliveryOkCount++;
    lastDeliveryOkMs = nowMs;
  } else {
    espNowDeliveryFailCount++;
    lastDeliveryFailMs = nowMs;
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  handleEspNowSendResult(status);
}
#else
void onEspNowDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  handleEspNowSendResult(status);
}
#endif

int16_t clampInt16(long value) {
  if (value > 32767) {
    return 32767;
  }
  if (value < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(value);
}

int32_t clampInt32(double value) {
  if (value > 2147483647.0) {
    return 2147483647;
  }
  if (value < -2147483648.0) {
    return -2147483647 - 1;
  }
  return static_cast<int32_t>(lround(value));
}

bool writeI2CRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

float rawAccelToG(int16_t rawValue) {
  return static_cast<float>(rawValue) / MPU6050_ACCEL_LSB_PER_G;
}

float rawGyroToDps(int16_t rawValue) {
  return static_cast<float>(rawValue) / MPU6050_GYRO_LSB_PER_DPS;
}

bool initMPU6050At(uint8_t address) {
  bool ok = writeI2CRegister(address, 0x6B, 0x00);  // Wake up MPU6050.
  delay(100);

  if (ok) {
    ok = writeI2CRegister(address, 0x1C, 0x00) &&  // Accelerometer +/-2g.
         writeI2CRegister(address, 0x1B, 0x00);    // Gyro +/-250 dps.
  }

  return ok;
}

bool initMPU6050() {
  bool ok = initMPU6050At(MPU6050_ADDR_PRIMARY);
  if (ok) {
    mpu6050Addr = MPU6050_ADDR_PRIMARY;
  } else {
    ok = initMPU6050At(MPU6050_ADDR_SECONDARY);
    if (ok) {
      mpu6050Addr = MPU6050_ADDR_SECONDARY;
    }
  }

  if (ok) {
    statusPrintln("[WRISTBAND] MPU6050 Ready at 0x" + String(mpu6050Addr, HEX));
  } else {
    statusPrintln("[WRISTBAND] MPU6050 Not Found at 0x68 or 0x69");
  }

  delay(100);
  return ok;
}

bool readMPU6050() {
  Wire.beginTransmission(mpu6050Addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t bytesRead = Wire.requestFrom(static_cast<uint8_t>(mpu6050Addr),
                                      static_cast<size_t>(14),
                                      true);
  if (bytesRead < 14 || Wire.available() < 14) {
    return false;
  }

  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  return true;
}

void calibrateIMU() {
  statusPrintln("[WRISTBAND] Keep wristband still for 2 seconds for IMU calibration...");

  double sumAx = 0.0;
  double sumAy = 0.0;
  double sumAz = 0.0;
  double sumGx = 0.0;
  double sumGy = 0.0;
  double sumGz = 0.0;
  uint16_t count = 0;
  uint32_t startMs = millis();

  while (millis() - startMs < 2000) {
    if (readMPU6050()) {
      sumAx += rawAccelToG(AccX);
      sumAy += rawAccelToG(AccY);
      sumAz += rawAccelToG(AccZ);
      sumGx += rawGyroToDps(GyroX);
      sumGy += rawGyroToDps(GyroY);
      sumGz += rawGyroToDps(GyroZ);
      count++;
    }
    delay(5);
  }

  if (count == 0) {
    statusPrintln("[WRISTBAND] IMU calibration skipped; no samples.");
    return;
  }

  float avgAx = sumAx / count;
  float avgAy = sumAy / count;
  float avgAz = sumAz / count;

  accelOffsetX = avgAx;
  accelOffsetY = avgAy;
  // Preserve the gravity component on Z instead of subtracting it away.
  accelOffsetZ = avgAz - (avgAz >= 0.0f ? 1.0f : -1.0f);
  gyroOffsetX = sumGx / count;
  gyroOffsetY = sumGy / count;
  gyroOffsetZ = sumGz / count;

  statusPrintln("[WRISTBAND] IMU calibration complete, samples=" + String(count));
  statusPrintln("[WRISTBAND] accel offsets g x/y/z=" + String(accelOffsetX, 4) +
                "/" + String(accelOffsetY, 4) + "/" + String(accelOffsetZ, 4));
  statusPrintln("[WRISTBAND] gyro offsets dps x/y/z=" + String(gyroOffsetX, 4) +
                "/" + String(gyroOffsetY, 4) + "/" + String(gyroOffsetZ, 4));
}

void updateIMU() {
  if (!imuReady) {
    imuUpdateMissCount++;
    return;
  }
  if (!readMPU6050()) {
    imuUpdateMissCount++;
    return;
  }
  imuUpdateOkCount++;

  accelX = rawAccelToG(AccX) - accelOffsetX;
  accelY = rawAccelToG(AccY) - accelOffsetY;
  accelZ = rawAccelToG(AccZ) - accelOffsetZ;
  gyroX = rawGyroToDps(GyroX) - gyroOffsetX;
  gyroY = rawGyroToDps(GyroY) - gyroOffsetY;
  gyroZ = rawGyroToDps(GyroZ) - gyroOffsetZ;

  pitchDeg = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;
  rollDeg = atan2f(accelY, accelZ) * 180.0f / PI;
}

void sendPacketToAntenna(const uint8_t *data, size_t len, const char *label) {
  // Terminate I2C while ESP-NOW transmits to reduce RF/I2C trace coupling.
  Wire.end();
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  esp_err_t result = esp_now_send(ANTENNA_MAC_PLACEHOLDER, data, len);
  if (result != ESP_OK) {
    espNowSendFailCount++;
    lastSendFailMs = millis();
    statusPrintln("[WRISTBAND] ESP-NOW " + String(label) + " send failed, err=" + String(result));
  } else {
    espNowSendOkCount++;
    lastSendOkMs = millis();
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
}

void sendWristbandData() {
  WristbandDataPacket packet = {};
  fillHeader(packet.header,
             MSG_WRISTBAND_DATA,
             DEVICE_WRISTBAND,
             ++wristSequence,
             millis(),
             false);

  packet.accel_mg_x = clampInt16(lround(accelX * 1000.0f));
  packet.accel_mg_y = clampInt16(lround(accelY * 1000.0f));
  packet.accel_mg_z = clampInt16(lround(accelZ * 1000.0f));
  packet.gyro_mdps_x = clampInt32(gyroX * 1000.0);
  packet.gyro_mdps_y = clampInt32(gyroY * 1000.0);
  packet.gyro_mdps_z = clampInt32(gyroZ * 1000.0);
  packet.pitch_cdeg = clampInt16(lround(pitchDeg * 100.0f));
  packet.roll_cdeg = clampInt16(lround(rollDeg * 100.0f));

  sendPacketToAntenna(reinterpret_cast<const uint8_t *>(&packet),
                      sizeof(packet),
                      "wristband data");
}

void sendBatteryStatus() {
  BatteryStatusPacket packet = {};
  fillHeader(packet.header,
             MSG_BATTERY_STATUS,
             DEVICE_WRISTBAND,
             ++wristSequence,
             millis(),
             false);
  packet.battery_mv = batterySenseValid ? static_cast<uint16_t>(lroundf(batteryVoltage * 1000.0f)) : 0;
  packet.battery_percent = batterySenseValid ? batteryPercent : 0;
  packet.battery_valid = batterySenseValid ? 1 : 0;
  packet.battery_adc_raw = batteryAdcRaw;

  sendPacketToAntenna(reinterpret_cast<const uint8_t *>(&packet),
                      sizeof(packet),
                      "battery status");
}

void maybeSendBatteryStatus() {
  if (lastBatteryReadMs == 0 || !antennaConnected || !batteryReportScheduled) {
    return;
  }

  uint32_t nowMs = millis();
  if (static_cast<int32_t>(nowMs - nextBatteryReportMs) >= 0) {
    lastBatteryReportMs = nowMs;
    nextBatteryReportMs = nowMs + BATTERY_REPORT_INTERVAL_MS;
    sendBatteryStatus();
  }
}

bool initializeEspNowStack() {
  if (esp_now_init() != ESP_OK) {
    statusPrintln("[WRISTBAND] ESP-NOW init failed.");
    return false;
  }
  statusPrintln("[WRISTBAND] ESP-NOW init ok.");
  esp_now_register_send_cb(onEspNowDataSent);
  esp_now_register_recv_cb(onEspNowDataRecv);
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
  return true;
}

void restoreEspNowAfterOtaFailure() {
  WiFi.disconnect(true);
  delay(100);
  configureWiFiChannel();
  initializeEspNowStack();
  antennaConnected = false;
  batteryReportScheduled = false;
  nextBatteryReportMs = 0;
}

void blinkOtaLed(uint32_t nowMs) {
  setStatusLed((nowMs / 100) % 2 == 0);
}

void performOtaUpdate(const OtaStartPacket &packet) {
  otaInProgress = true;
  statusPrintln("[WRISTBAND] OTA requested, url=" + String(packet.url));

  esp_now_deinit();
  Wire.end();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(packet.ssid, packet.password);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 60000UL) {
    blinkOtaLed(millis());
    delay(50);
  }

  if (WiFi.status() != WL_CONNECTED) {
    otaFailCount++;
    statusPrintln("[WRISTBAND] OTA WiFi connect failed.");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    restoreEspNowAfterOtaFailure();
    otaInProgress = false;
    return;
  }

  statusPrintln("[WRISTBAND] OTA WiFi connected, IP=" + WiFi.localIP().toString());
  WiFiClient client;
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.onProgress([](int current, int total) {
    static uint32_t lastProgressMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastProgressMs >= 500) {
      lastProgressMs = nowMs;
      blinkOtaLed(nowMs);
      if (total > 0) {
        statusPrintln("[WRISTBAND] OTA progress " + String((current * 100) / total) + "%");
      }
    }
  });

  t_httpUpdate_return result = httpUpdate.update(client, String(packet.url));
  if (result == HTTP_UPDATE_OK) {
    statusPrintln("[WRISTBAND] OTA update ok; rebooting.");
    delay(100);
    ESP.restart();
  }

  otaFailCount++;
  statusPrintln("[WRISTBAND] OTA update failed, err=" +
                String(httpUpdate.getLastError()) + " " +
                httpUpdate.getLastErrorString());
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
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

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  setStatusLed(false);
  runStatusLedSelfTest();
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  Serial.begin(AIRTRIXX_SERIAL_BAUD);
  delay(200);
  statusPrintln("");
  statusPrintln("[WRISTBAND] AirTrixx Wristband ESP32-C3 booting...");
  statusPrintln("[WRISTBAND] Serial baud=" + String(AIRTRIXX_SERIAL_BAUD));

  statusPrintln("[WRISTBAND] Starting sensor I2C: SDA=" + String(SDA_PIN) +
                " SCL=" + String(SCL_PIN) + ", MPU6050=0x68/0x69");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  imuReady = initMPU6050();

  if (imuReady) {
    calibrateIMU();
  } else {
    statusPrintln("[WRISTBAND] IMU calibration skipped; MPU6050 is not ready.");
  }

  configureWiFiChannel();
  if (!initializeEspNowStack()) {
    return;
  }
  statusPrintln("[WRISTBAND] Setup complete.");
}

void printConnectionStatus() {
  if (!STATUS_SERIAL) {
    return;
  }

  uint32_t nowMs = millis();
  Serial.print("[WRISTBAND] status t=");
  Serial.print(nowMs);
  Serial.print("ms, imu=");
  Serial.print(imuReady ? "ok" : "not_ready");
  Serial.print(", imu_addr=0x");
  Serial.print(mpu6050Addr, HEX);
  Serial.print(", peer=");
  Serial.print(esp_now_is_peer_exist(ANTENNA_MAC_PLACEHOLDER) ? "yes" : "no");
  Serial.print(", send_ok=");
  Serial.print(espNowSendOkCount);
  Serial.print(", send_fail=");
  Serial.print(espNowSendFailCount);
  Serial.print(", delivery_ok=");
  Serial.print(espNowDeliveryOkCount);
  Serial.print(", delivery_fail=");
  Serial.print(espNowDeliveryFailCount);
  Serial.print(", antenna_connected=");
  Serial.print(antennaConnected ? "yes" : "no");
  Serial.print(", battery_v=");
  Serial.print(batteryVoltage, 3);
  Serial.print(", battery_pct=");
  Serial.print(batterySenseValid ? static_cast<int>(batteryPercent) : -1);
  Serial.print(", battery_low=");
  Serial.print(batteryLow ? "yes" : "no");
  Serial.print(", battery_sense=");
  Serial.print(batterySenseValid ? "valid" : "invalid");
  Serial.print(", battery_adc=");
  Serial.print(batteryAdcRaw);
  Serial.print(", battery_next_report_age_ms=");
  if (!batteryReportScheduled) {
    Serial.print(-1);
  } else {
    Serial.print(static_cast<int32_t>(nowMs - nextBatteryReportMs));
  }
  Serial.print(", last_send_ok_age_ms=");
  Serial.print(lastSendOkMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendOkMs));
  Serial.print(", last_send_fail_age_ms=");
  Serial.print(lastSendFailMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendFailMs));
  Serial.print(", last_delivery_ok_age_ms=");
  Serial.print(lastDeliveryOkMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastDeliveryOkMs));
  Serial.print(", last_delivery_fail_age_ms=");
  Serial.print(lastDeliveryFailMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastDeliveryFailMs));
  Serial.print(", seq=");
  Serial.print(wristSequence);
  Serial.print(", imu_updates ok/miss=");
  Serial.print(imuUpdateOkCount);
  Serial.print("/");
  Serial.print(imuUpdateMissCount);
  Serial.print(", ota_rx/fail=");
  Serial.print(otaStartRxCount);
  Serial.print("/");
  Serial.print(otaFailCount);
  Serial.print(", raw_acc x/y/z=");
  Serial.print(AccX);
  Serial.print("/");
  Serial.print(AccY);
  Serial.print("/");
  Serial.print(AccZ);
  Serial.print(", raw_gyro x/y/z=");
  Serial.print(GyroX);
  Serial.print("/");
  Serial.print(GyroY);
  Serial.print("/");
  Serial.print(GyroZ);
  Serial.print(", accel_g x/y/z=");
  Serial.print(accelX, 3);
  Serial.print("/");
  Serial.print(accelY, 3);
  Serial.print("/");
  Serial.print(accelZ, 3);
  Serial.print(", gyro_dps x/y/z=");
  Serial.print(gyroX, 2);
  Serial.print("/");
  Serial.print(gyroY, 2);
  Serial.print("/");
  Serial.print(gyroZ, 2);
  Serial.print(", pitch/roll=");
  Serial.print(pitchDeg, 1);
  Serial.print("/");
  Serial.println(rollDeg, 1);
}

void loop() {
  processPendingOta();
  if (otaInProgress) {
    return;
  }

  updateIMU();
  updateBatteryStatus();

  uint32_t nowMs = millis();
  const uint32_t intervalMs = 1000UL / WRISTBAND_REPORT_HZ;
  if (nowMs - lastReportMs >= intervalMs) {
    lastReportMs = nowMs;
    sendWristbandData();
  }
  refreshAntennaConnectionStatus(nowMs);
  maybeSendBatteryStatus();

  updateStatusLed();

  if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintMs = nowMs;
    printConnectionStatus();
  }
}
