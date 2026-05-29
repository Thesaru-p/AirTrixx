#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L1X.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

static Adafruit_PWMServoDriver pwm(PCA9685_ADDR, Wire);
static VL53L1X tofLeft;
static VL53L1X tofRight;

static bool leftToFReady = false;
static bool rightToFReady = false;
static uint16_t lastLeftToFmm = 0;
static uint16_t lastRightToFmm = 0;
static uint8_t currentActivePair = ACTIVE_PAIR_NONE;
static uint16_t batteryAdcRaw = 0;
static float batteryVoltage = 0.0f;
static uint8_t batteryPercent = 0;
static bool batteryValid = false;

static uint16_t camDockSequence = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t lastBatteryReadMs = 0;
static uint32_t espNowSendOkCount = 0;
static uint32_t espNowSendFailCount = 0;
static uint32_t lastSendOkMs = 0;
static uint32_t lastSendFailMs = 0;
static uint32_t servoCommandRxCount = 0;
static uint32_t badPacketCount = 0;
static uint32_t tofLeftTimeoutCount = 0;
static uint32_t tofRightTimeoutCount = 0;
static uint32_t otaStartRxCount = 0;
static uint32_t otaFailCount = 0;
static uint16_t lastServoCommandSequence = 0;

static ServoCommandPacket pendingCommand = {};
static bool hasPendingCommand = false;
static portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
static OtaStartPacket pendingOtaPacket = {};
static bool hasPendingOta = false;
static bool otaInProgress = false;
static portMUX_TYPE otaMux = portMUX_INITIALIZER_UNLOCKED;

// Cam Dock is not the USB JSON source, so local status logs are safe here.
static const bool STATUS_SERIAL = true;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;
static const uint32_t BATTERY_READ_INTERVAL_MS = 500;
static const uint8_t BATTERY_ADC_SAMPLES = 8;
static const float BATTERY_DIVIDER_RATIO = 147.0f / 47.0f;
static const float BATTERY_VALID_MIN_V = 4.0f;
static const float BATTERY_EMPTY_V = 6.0f;
static const float BATTERY_FULL_V = 8.4f;

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

uint8_t batteryPercentFromVoltage(float voltage) {
  struct BatteryPoint {
    float voltage;
    uint8_t percent;
  };
  static const BatteryPoint curve[] = {
    {BATTERY_EMPTY_V, 0},
    {6.60f, 5},
    {7.00f, 15},
    {7.20f, 25},
    {7.40f, 40},
    {7.60f, 55},
    {7.80f, 70},
    {8.00f, 82},
    {8.20f, 92},
    {BATTERY_FULL_V, 100},
  };

  if (voltage <= curve[0].voltage) {
    return 0;
  }
  const size_t lastIndex = (sizeof(curve) / sizeof(curve[0])) - 1;
  if (voltage >= curve[lastIndex].voltage) {
    return 100;
  }
  for (size_t i = 1; i <= lastIndex; ++i) {
    if (voltage <= curve[i].voltage) {
      float spanV = curve[i].voltage - curve[i - 1].voltage;
      float ratio = (voltage - curve[i - 1].voltage) / spanV;
      float percent = curve[i - 1].percent +
                      ratio * (curve[i].percent - curve[i - 1].percent);
      return static_cast<uint8_t>(constrain(lroundf(percent), 0L, 100L));
    }
  }
  return 100;
}

void updateBatteryStatus(bool force = false) {
  uint32_t nowMs = millis();
  if (!force && nowMs - lastBatteryReadMs < BATTERY_READ_INTERVAL_MS) {
    return;
  }
  lastBatteryReadMs = nowMs;

  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
    rawSum += analogRead(CAMDOCK_BATTERY_ADC_PIN);
    mvSum += analogReadMilliVolts(CAMDOCK_BATTERY_ADC_PIN);
    delayMicroseconds(200);
  }

  batteryAdcRaw = rawSum / BATTERY_ADC_SAMPLES;
  float pinVoltage = (static_cast<float>(mvSum) / BATTERY_ADC_SAMPLES) / 1000.0f;
  batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;
  batteryValid = batteryVoltage >= BATTERY_VALID_MIN_V;
  batteryPercent = batteryValid ? batteryPercentFromVoltage(batteryVoltage) : 0;
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

  Serial.print("[CAMDOCK] WiFi mode STA, MAC=");
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
  statusPrintln("[CAMDOCK] Turning WiFi on in STA mode...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  printWiFiStatus();
}

bool addEspNowPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    statusPrintln("[CAMDOCK] ESP-NOW antenna peer already exists.");
    return true;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    statusPrintln("[CAMDOCK] ESP-NOW add antenna peer failed, err=" + String(result));
    return false;
  }
  statusPrintln("[CAMDOCK] ESP-NOW antenna peer added.");
  return true;
}

void selectMuxChannel(uint8_t ch) {
  if (ch > 7) {
    return;
  }
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(1 << ch);
  Wire1.endTransmission();
}

bool initToFSensor(VL53L1X &sensor, uint8_t muxChannel) {
  selectMuxChannel(muxChannel);
  delay(5);

  // Pololu VL53L1X library API. If you use another VL53L1X library, adapt
  // only this function and readToFLeft/readToFRight below.
  sensor.setBus(&Wire1);
  sensor.setTimeout(50);
  if (!sensor.init()) {
    statusPrintln("[CAMDOCK] VL53L1X init failed on mux channel " + String(muxChannel));
    return false;
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(33000);
  sensor.startContinuous(33);
  statusPrintln("[CAMDOCK] VL53L1X ready on mux channel " + String(muxChannel));
  return true;
}

uint16_t readToFLeft() {
  if (!leftToFReady) {
    return 0;
  }
  selectMuxChannel(CH_LEFT_TOF);
  uint16_t mm = tofLeft.read(false);
  if (tofLeft.timeoutOccurred()) {
    tofLeftTimeoutCount++;
    return lastLeftToFmm;
  }
  lastLeftToFmm = mm;
  return mm;
}

uint16_t readToFRight() {
  if (!rightToFReady) {
    return 0;
  }
  selectMuxChannel(CH_RIGHT_TOF);
  uint16_t mm = tofRight.read(false);
  if (tofRight.timeoutOccurred()) {
    tofRightTimeoutCount++;
    return lastRightToFmm;
  }
  lastRightToFmm = mm;
  return mm;
}

void disableServo(uint8_t channel) {
  pwm.setPWM(channel, 0, 0);
}

void setServoPulse(uint8_t channel, uint16_t pulse) {
  if (pulse == 0) {
    disableServo(channel);
    return;
  }
  if (pulse > 4095) {
    pulse = 4095;
  }
  pwm.setPWM(channel, 0, pulse);
}

void disableUnusedServos(uint8_t activePair) {
  if (activePair != ACTIVE_PAIR_RIGHT && activePair != ACTIVE_PAIR_HANDS &&
      activePair != ACTIVE_PAIR_DOCK) {
    disableServo(CH_R_PAN);
    disableServo(CH_R_TILT);
  }
  if (activePair != ACTIVE_PAIR_LEFT && activePair != ACTIVE_PAIR_HANDS &&
      activePair != ACTIVE_PAIR_DOCK) {
    disableServo(CH_L_PAN);
    disableServo(CH_L_TILT);
  }
  if (activePair != ACTIVE_PAIR_CAMERA && activePair != ACTIVE_PAIR_DOCK) {
    disableServo(CH_CAM_PAN);
    disableServo(CH_CAM_TILT);
  }
}

void applyServoIfActive(uint8_t mask,
                        uint8_t bit,
                        uint8_t channel,
                        uint16_t pulse) {
  if (mask & bit) {
    setServoPulse(channel, pulse);
  }
}

void applyServoCommand(const ServoCommandPacket &command) {
  uint8_t activePair = command.active_pair;
  uint8_t activeMask = command.active_mask;
  uint8_t disableUnused = command.disable_unused;
  uint16_t sequence = command.header.sequence;
  uint16_t rPan = command.r_pan;
  uint16_t rTilt = command.r_tilt;
  uint16_t lPan = command.l_pan;
  uint16_t lTilt = command.l_tilt;
  uint16_t camPan = command.cam_pan;
  uint16_t camTilt = command.cam_tilt;

  currentActivePair = activePair;
  lastServoCommandSequence = sequence;

  if (servoCommandRxCount <= 5 || (servoCommandRxCount % 30) == 0) {
    statusPrintln("[CAMDOCK] Servo command rx seq=" + String(sequence) +
                  ", active_pair=" + String(activePairToString(activePair)) +
                  ", mask=0x" + String(activeMask, HEX));
  }

  if (currentActivePair == ACTIVE_PAIR_NONE) {
    disableUnusedServos(ACTIVE_PAIR_NONE);
    return;
  }

  applyServoIfActive(activeMask, SERVO_MASK_R_PAN, CH_R_PAN, rPan);
  applyServoIfActive(activeMask, SERVO_MASK_R_TILT, CH_R_TILT, rTilt);
  applyServoIfActive(activeMask, SERVO_MASK_L_PAN, CH_L_PAN, lPan);
  applyServoIfActive(activeMask, SERVO_MASK_L_TILT, CH_L_TILT, lTilt);
  applyServoIfActive(activeMask, SERVO_MASK_CAM_PAN, CH_CAM_PAN, camPan);
  applyServoIfActive(activeMask, SERVO_MASK_CAM_TILT, CH_CAM_TILT, camTilt);

  if (disableUnused) {
    disableUnusedServos(activePair);
  }
}

void centerAllBracketsAtStartup() {
  statusPrintln("[CAMDOCK] Centering all brackets at startup...");
  currentActivePair = ACTIVE_PAIR_DOCK;
  setServoPulse(CH_CAM_PAN, DEFAULT_CAM_PAN_CENTER);
  setServoPulse(CH_CAM_TILT, DEFAULT_CAM_TILT_CENTER);
  setServoPulse(CH_R_PAN, DEFAULT_R_PAN_CENTER);
  setServoPulse(CH_R_TILT, DEFAULT_R_TILT_CENTER);
  setServoPulse(CH_L_PAN, DEFAULT_L_PAN_CENTER);
  setServoPulse(CH_L_TILT, DEFAULT_L_TILT_CENTER);
  delay(700);
  disableUnusedServos(ACTIVE_PAIR_NONE);
  currentActivePair = ACTIVE_PAIR_NONE;
  statusPrintln("[CAMDOCK] Startup bracket centering complete; servos disabled.");
}

void sendCamDockData() {
  CamDockDataPacket packet = {};
  fillHeader(packet.header,
             MSG_CAMDOCK_DATA,
             DEVICE_CAMDOCK,
             ++camDockSequence,
             millis(),
             false);
  packet.left_tof_mm = lastLeftToFmm;
  packet.right_tof_mm = lastRightToFmm;
  packet.battery_mv = batteryValid ? static_cast<uint16_t>(lroundf(batteryVoltage * 1000.0f)) : 0;
  packet.battery_adc_raw = batteryAdcRaw;
  packet.battery_percent = batteryValid ? batteryPercent : 0;
  packet.battery_valid = batteryValid ? 1 : 0;
  packet.active_target = currentActivePair;

  esp_err_t result = esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                                  reinterpret_cast<uint8_t *>(&packet),
                                  sizeof(packet));
  if (result != ESP_OK) {
    espNowSendFailCount++;
    lastSendFailMs = millis();
    statusPrintln("[CAMDOCK] ESP-NOW camdock data send failed, err=" + String(result));
  } else {
    espNowSendOkCount++;
    lastSendOkMs = millis();
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

  if (header.msg_type == MSG_SERVO_COMMAND && len == static_cast<int>(sizeof(ServoCommandPacket))) {
    ServoCommandPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    portENTER_CRITICAL(&commandMux);
    pendingCommand = packet;
    hasPendingCommand = true;
    servoCommandRxCount++;
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

void setup() {
  Serial.begin(AIRTRIXX_SERIAL_BAUD);
  delay(200);
  statusPrintln("");
  statusPrintln("[CAMDOCK] AirTrixx Cam Dock ESP32-S3 booting...");
  statusPrintln("[CAMDOCK] Serial baud=" + String(AIRTRIXX_SERIAL_BAUD));
  pinMode(CAMDOCK_BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(CAMDOCK_BATTERY_ADC_PIN, ADC_11db);
  updateBatteryStatus(true);
  statusPrintln("[CAMDOCK] Battery sense GPIO=" + String(CAMDOCK_BATTERY_ADC_PIN) +
                ", divider=100k/47k, voltage=" + String(batteryVoltage, 3) +
                "V, percent=" + String(batteryValid ? static_cast<int>(batteryPercent) : -1));

  statusPrintln("[CAMDOCK] Starting I2C buses: servo SDA=" + String(SDA_SERVO) +
                " SCL=" + String(SCL_SERVO) + ", tof SDA=" + String(SDA_TOF) +
                " SCL=" + String(SCL_TOF));
  Wire.begin(SDA_SERVO, SCL_SERVO);
  Wire1.begin(SDA_TOF, SCL_TOF);

  statusPrintln("[CAMDOCK] Starting PCA9685 at address 0x" + String(PCA9685_ADDR, HEX));
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  centerAllBracketsAtStartup();

  leftToFReady = initToFSensor(tofLeft, CH_LEFT_TOF);
  rightToFReady = initToFSensor(tofRight, CH_RIGHT_TOF);

  configureWiFiChannel();
  if (esp_now_init() != ESP_OK) {
    statusPrintln("[CAMDOCK] ESP-NOW init failed.");
    return;
  }
  statusPrintln("[CAMDOCK] ESP-NOW init ok.");
  esp_now_register_recv_cb(onDataRecv);
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
  statusPrintln("[CAMDOCK] Setup complete.");
}

bool initializeEspNowStack() {
  if (esp_now_init() != ESP_OK) {
    statusPrintln("[CAMDOCK] ESP-NOW init failed.");
    return false;
  }
  statusPrintln("[CAMDOCK] ESP-NOW init ok.");
  esp_now_register_recv_cb(onDataRecv);
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
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
  disableUnusedServos(ACTIVE_PAIR_NONE);
  currentActivePair = ACTIVE_PAIR_NONE;
  statusPrintln("[CAMDOCK] OTA requested, url=" + String(packet.url));

  esp_now_deinit();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(packet.ssid, packet.password);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 60000UL) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    otaFailCount++;
    statusPrintln("[CAMDOCK] OTA WiFi connect failed.");
    restoreEspNowAfterOtaFailure();
    otaInProgress = false;
    return;
  }

  statusPrintln("[CAMDOCK] OTA WiFi connected, IP=" + WiFi.localIP().toString());
  WiFiClient client;
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.onProgress([](int current, int total) {
    static uint32_t lastProgressMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastProgressMs >= 500) {
      lastProgressMs = nowMs;
      if (total > 0) {
        statusPrintln("[CAMDOCK] OTA progress " + String((current * 100) / total) + "%");
      }
    }
  });

  t_httpUpdate_return result = httpUpdate.update(client, String(packet.url));
  if (result == HTTP_UPDATE_OK) {
    statusPrintln("[CAMDOCK] OTA update ok; rebooting.");
    delay(100);
    ESP.restart();
  }

  otaFailCount++;
  statusPrintln("[CAMDOCK] OTA update failed, err=" +
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
  Serial.print("[CAMDOCK] status t=");
  Serial.print(nowMs);
  Serial.print("ms, peer=");
  Serial.print(esp_now_is_peer_exist(ANTENNA_MAC_PLACEHOLDER) ? "yes" : "no");
  Serial.print(", send_ok=");
  Serial.print(espNowSendOkCount);
  Serial.print(", send_fail=");
  Serial.print(espNowSendFailCount);
  Serial.print(", last_send_ok_age_ms=");
  Serial.print(lastSendOkMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendOkMs));
  Serial.print(", last_send_fail_age_ms=");
  Serial.print(lastSendFailMs == 0 ? -1 : static_cast<int32_t>(nowMs - lastSendFailMs));
  Serial.print(", servo_rx=");
  Serial.print(servoCommandRxCount);
  Serial.print(", last_servo_seq=");
  Serial.print(lastServoCommandSequence);
  Serial.print(", bad_packets=");
  Serial.print(badPacketCount);
  Serial.print(", ota_rx/fail=");
  Serial.print(otaStartRxCount);
  Serial.print("/");
  Serial.print(otaFailCount);
  Serial.print(", tof_left=");
  Serial.print(leftToFReady ? lastLeftToFmm : 0);
  Serial.print("mm, tof_right=");
  Serial.print(rightToFReady ? lastRightToFmm : 0);
  Serial.print("mm, tof_timeouts L/R=");
  Serial.print(tofLeftTimeoutCount);
  Serial.print("/");
  Serial.print(tofRightTimeoutCount);
  Serial.print(", battery_v=");
  Serial.print(batteryVoltage, 3);
  Serial.print(", battery_pct=");
  Serial.print(batteryValid ? static_cast<int>(batteryPercent) : -1);
  Serial.print(", battery_adc=");
  Serial.print(batteryAdcRaw);
  Serial.print(", active_pair=");
  Serial.println(activePairToString(currentActivePair));
}

void loop() {
  processPendingOta();
  if (otaInProgress) {
    return;
  }

  bool shouldApply = false;
  ServoCommandPacket command = {};
  portENTER_CRITICAL(&commandMux);
  if (hasPendingCommand) {
    command = pendingCommand;
    hasPendingCommand = false;
    shouldApply = true;
  }
  portEXIT_CRITICAL(&commandMux);

  if (shouldApply) {
    applyServoCommand(command);
  }

  uint32_t nowMs = millis();
  updateBatteryStatus();
  const uint32_t intervalMs = 1000UL / CAMDOCK_REPORT_HZ;
  if (nowMs - lastReportMs >= intervalMs) {
    lastReportMs = nowMs;
    lastLeftToFmm = readToFLeft();
    lastRightToFmm = readToFRight();
    sendCamDockData();
  }

  if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintMs = nowMs;
    printConnectionStatus();
  }
}
