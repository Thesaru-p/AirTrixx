#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#if __has_include("../../shared/AirTrixxConfig.h")
#include "../../shared/AirTrixxConfig.h"
#include "../../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

static const uint8_t CHANNEL_COUNT = 4;

// Pins
static const uint8_t GATE_PINS[CHANNEL_COUNT] = {11, 4, 5, 6};
static const uint8_t ENC_CLK = 7;
static const uint8_t ENC_DT = 1;
static const uint8_t ENC_SW = 2;
static const uint8_t TEMP_BUS = 3;
static const uint8_t I2C_SDA = 10;
static const uint8_t I2C_SCL = 9;

// Electrical constants
static const float BATT_IR_OHMS = 0.41f;
static const float VOLT_OFFSET = 0.0f;
static const float NO_BATTERY_V = 2.50f;
static const float IDLE_FULL_V = 4.10f;
static const float IDLE_CURRENT_MA = 15.0f;
static const float FULL_RELEASE_V = 4.18f;
static const float FULL_RELEASE_CURRENT_MA = 50.0f;
static const float THERMAL_CUTOFF_C = 50.0f;
static const float MIN_MAH_CURRENT_MA = 20.0f;

static const uint32_t SENSOR_INTERVAL_MS = 500;
static const uint32_t DISPLAY_INTERVAL_MS = 250;
static const uint32_t STATUS_INTERVAL_MS = 2000;
static const uint32_t REPORT_INTERVAL_MS = 1000UL / CHARGING_DOCK_REPORT_HZ;
static const uint32_t BUTTON_DEBOUNCE_MS = 250;
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.

// Gate MOSFETs are active-low in the supplied sketch.
static const uint8_t GATE_ON_LEVEL = LOW;
static const uint8_t GATE_OFF_LEVEL = HIGH;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_INA219 ina1(0x45);
Adafruit_INA219 ina2(0x44);
Adafruit_INA219 ina3(0x41);
Adafruit_INA219 ina4(0x40);
Adafruit_INA219 *sensors[CHANNEL_COUNT] = {&ina1, &ina2, &ina3, &ina4};
OneWire oneWire(TEMP_BUS);
DallasTemperature temps(&oneWire);

static bool displayReady = false;
static bool inaReady[CHANNEL_COUNT] = {false, false, false, false};
static bool gateEnabled[CHANNEL_COUNT] = {false, false, false, false};
static bool tempValid[CHANNEL_COUNT] = {false, false, false, false};
static bool noBattery[CHANNEL_COUNT] = {true, true, true, true};
static bool fullOrIdle[CHANNEL_COUNT] = {false, false, false, false};
static bool hotCutoff[CHANNEL_COUNT] = {false, false, false, false};

static float mah[CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float curTemp[CHANNEL_COUNT] = {DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C,
                                       DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C};
static float smoothedV[CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float batteryV[CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float currentmA[CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

static int activeTab = 0;
static int priorityCH = -1;  // -1 = charge all valid channels, 0-3 = priority channel.
static int lastClk = HIGH;
static bool lastButtonState = HIGH;

static uint32_t lastTickMs = 0;
static uint32_t lastSensorMs = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastButtonMs = 0;
static uint16_t chargingDockSequence = 0;
static uint32_t espNowSendOkCount = 0;
static uint32_t espNowSendFailCount = 0;
static bool espNowReady = false;

static void printMacAddress(const uint8_t mac[6]) {
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

static void configureWiFiChannel() {
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
  Serial.print("[CHG] WiFi STA MAC=");
  printMacAddress(mac);
  Serial.print(", channel=");
  Serial.println(ESPNOW_CHANNEL);
}

static bool addEspNowPeer(const uint8_t mac[6]) {
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
    Serial.println("[CHG] ESP-NOW add antenna peer failed, err=" + String(result));
    return false;
  }
  return true;
}

static void initializeEspNow() {
  configureWiFiChannel();
  if (esp_now_init() != ESP_OK) {
    Serial.println("[CHG] ESP-NOW init failed; local charging still enabled");
    espNowReady = false;
    return;
  }
  espNowReady = addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);
  Serial.println(espNowReady ? "[CHG] ESP-NOW antenna peer ready" :
                               "[CHG] ESP-NOW antenna peer unavailable");
}

static void setGate(uint8_t ch, bool enabled) {
  gateEnabled[ch] = enabled;
  digitalWrite(GATE_PINS[ch], enabled ? GATE_ON_LEVEL : GATE_OFF_LEVEL);
}

static void allGatesOff() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    setGate(i, false);
  }
}

static float readCompensatedVoltage(uint8_t ch) {
  if (!inaReady[ch]) {
    return 0.0f;
  }

  float rawV = sensors[ch]->getBusVoltage_V();
  float currentA = sensors[ch]->getCurrent_mA() / 1000.0f;
  float compensated = rawV - (currentA * BATT_IR_OHMS) + VOLT_OFFSET;

  if (smoothedV[ch] < 0.5f) {
    smoothedV[ch] = compensated;
  } else {
    smoothedV[ch] = (smoothedV[ch] * 0.8f) + (compensated * 0.2f);
  }

  return smoothedV[ch];
}

static int batteryPercent(float volts) {
  if (volts <= 3.20f) {
    return 0;
  }
  if (volts >= 4.20f) {
    return 100;
  }
  return static_cast<int>(((volts - 3.20f) * 100.0f / 1.0f) + 0.5f);
}

static uint16_t clampUint16(float value) {
  if (value <= 0.0f) {
    return 0;
  }
  if (value >= 65535.0f) {
    return 65535;
  }
  return static_cast<uint16_t>(lroundf(value));
}

static int16_t clampInt16(float value) {
  if (value <= -32768.0f) {
    return -32768;
  }
  if (value >= 32767.0f) {
    return 32767;
  }
  return static_cast<int16_t>(lroundf(value));
}

static bool hasUsableTemp(uint8_t ch) {
  return tempValid[ch] && curTemp[ch] > -40.0f && curTemp[ch] < 125.0f;
}

static void updateSensorsAndChargeLogic() {
  uint32_t now = millis();
  float hours = 0.0f;
  if (lastTickMs != 0) {
    hours = static_cast<float>(now - lastTickMs) / 3600000.0f;
  }
  lastTickMs = now;

  temps.requestTemperatures();

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (inaReady[i]) {
      batteryV[i] = readCompensatedVoltage(i);
      currentmA[i] = sensors[i]->getCurrent_mA();
    } else {
      batteryV[i] = 0.0f;
      currentmA[i] = 0.0f;
    }

    curTemp[i] = temps.getTempCByIndex(i);
    tempValid[i] = curTemp[i] != DEVICE_DISCONNECTED_C;

    noBattery[i] = !inaReady[i] || batteryV[i] < NO_BATTERY_V;
    fullOrIdle[i] = inaReady[i] && batteryV[i] > IDLE_FULL_V && currentmA[i] < IDLE_CURRENT_MA;
    hotCutoff[i] = hasUsableTemp(i) && curTemp[i] > THERMAL_CUTOFF_C;

    if (gateEnabled[i] && currentmA[i] > MIN_MAH_CURRENT_MA && hours > 0.0f) {
      mah[i] += currentmA[i] * hours;
    }

    bool chargeAllowed = inaReady[i] && !noBattery[i] && !fullOrIdle[i] && !hotCutoff[i];
    bool priorityAllows = priorityCH == -1 || priorityCH == static_cast<int>(i);
    setGate(i, chargeAllowed && priorityAllows);

    if (priorityCH == static_cast<int>(i) &&
        ((batteryV[i] > FULL_RELEASE_V && currentmA[i] < FULL_RELEASE_CURRENT_MA) ||
         noBattery[i] || hotCutoff[i])) {
      priorityCH = -1;
    }
  }
}

static void handleEncoder() {
  int clk = digitalRead(ENC_CLK);
  if (clk != lastClk && clk == LOW) {
    if (digitalRead(ENC_DT) != clk) {
      ++activeTab;
    } else {
      --activeTab;
    }

    if (activeTab > static_cast<int>(CHANNEL_COUNT)) {
      activeTab = 0;
    } else if (activeTab < 0) {
      activeTab = CHANNEL_COUNT;
    }
  }
  lastClk = clk;

  bool buttonState = digitalRead(ENC_SW);
  uint32_t now = millis();
  if (lastButtonState == HIGH && buttonState == LOW &&
      now - lastButtonMs >= BUTTON_DEBOUNCE_MS) {
    lastButtonMs = now;
    if (activeTab > 0) {
      int selected = activeTab - 1;
      priorityCH = priorityCH == selected ? -1 : selected;
    } else {
      priorityCH = -1;
    }
  }
  lastButtonState = buttonState;
}

static void printChannelStatus(uint8_t ch) {
  if (!inaReady[ch]) {
    display.print("INA ERR");
  } else if (hotCutoff[ch]) {
    display.print("HOT ");
    display.print(curTemp[ch], 0);
    display.print("C");
  } else if (noBattery[ch]) {
    display.print("NO BATT");
  } else if (fullOrIdle[ch]) {
    display.print("FULL");
  } else {
    display.print(batteryV[ch], 2);
    display.print("V ");
    display.print(static_cast<int>(currentmA[ch]));
    display.print("mA");
  }
}

static void drawOverview() {
  display.setTextSize(1);
  display.setCursor(18, 0);
  display.println("SMART CHG DOCK");

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    display.setCursor(0, 16 + (i * 12));
    if (priorityCH == static_cast<int>(i)) {
      display.print(">");
    } else if (gateEnabled[i]) {
      display.print("*");
    } else {
      display.print(" ");
    }
    display.print("CH");
    display.print(i + 1);
    display.print(": ");
    printChannelStatus(i);
  }
}

static void drawChannelDetail(uint8_t ch) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("CHANNEL ");
  display.print(ch + 1);
  if (priorityCH == static_cast<int>(ch)) {
    display.print(" [PRI]");
  } else if (gateEnabled[ch]) {
    display.print(" [ON]");
  }
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 16);
  if (!inaReady[ch]) {
    display.print("INA ERR");
  } else if (noBattery[ch]) {
    display.print("NO BATT");
  } else if (fullOrIdle[ch]) {
    display.print("FULL");
  } else {
    display.print(batteryV[ch], 2);
    display.print("V");
  }

  display.drawRect(0, 34, 80, 8, SSD1306_WHITE);
  if (inaReady[ch] && !noBattery[ch]) {
    int pct = batteryPercent(batteryV[ch]);
    display.fillRect(1, 35, map(pct, 0, 100, 0, 78), 6, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(85, 16);
    display.print(pct);
    display.print("%");
  }

  display.setTextSize(1);
  display.setCursor(0, 46);
  display.print("Energy: ");
  display.print(static_cast<int>(mah[ch]));
  display.print(" mAh");

  display.setCursor(85, 46);
  if (tempValid[ch]) {
    display.print(curTemp[ch], 1);
    display.print("C");
  } else {
    display.print("--.-C");
  }

  display.setCursor(0, 56);
  display.print("BTN: SET PRIORITY");
}

static void updateDisplay() {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (activeTab == 0) {
    drawOverview();
  } else {
    drawChannelDetail(activeTab - 1);
  }

  display.display();
}

static void printStatus() {
  Serial.print("[CHG] tab=");
  Serial.print(activeTab);
  Serial.print(" priority=");
  Serial.print(priorityCH);
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(" ch");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(inaReady[i] ? "ok" : "ina_err");
    Serial.print(gateEnabled[i] ? "/on" : "/off");
    Serial.print("/");
    Serial.print(batteryV[i], 3);
    Serial.print("V/");
    Serial.print(currentmA[i], 0);
    Serial.print("mA/");
    if (tempValid[i]) {
      Serial.print(curTemp[i], 1);
      Serial.print("C");
    } else {
      Serial.print("temp_err");
    }
  }
  Serial.print(" espnow=");
  Serial.print(espNowReady ? "ready" : "off");
  Serial.print(" send_ok/fail=");
  Serial.print(espNowSendOkCount);
  Serial.print("/");
  Serial.print(espNowSendFailCount);
  Serial.println();
}

static void sendChargingDockStatus() {
  if (!espNowReady) {
    return;
  }

  ChargingDockStatusPacket packet = {};
  bool batteryLow = false;
  fillHeader(packet.header,
             MSG_CHARGING_DOCK_STATUS,
             DEVICE_CHARGING_DOCK,
             ++chargingDockSequence,
             millis(),
             false);
  packet.active_tab = static_cast<uint8_t>(activeTab);
  packet.priority_channel = static_cast<int8_t>(priorityCH);

  for (uint8_t i = 0; i < CHANNEL_COUNT && i < AIRTRIXX_CHARGING_DOCK_CHANNELS; ++i) {
    uint8_t bit = 1 << i;
    bool batteryPresent = inaReady[i] && !noBattery[i];
    if (inaReady[i]) {
      packet.ina_valid_mask |= bit;
    }
    if (batteryPresent) {
      packet.battery_present_mask |= bit;
    }
    if (gateEnabled[i]) {
      packet.charging_mask |= bit;
    }
    if (fullOrIdle[i]) {
      packet.full_mask |= bit;
    }
    if (hotCutoff[i]) {
      packet.hot_mask |= bit;
    }
    if (tempValid[i]) {
      packet.temp_valid_mask |= bit;
    }

    uint8_t percent = batteryPresent ? static_cast<uint8_t>(batteryPercent(batteryV[i])) : 0;
    packet.battery_percent[i] = percent;
    packet.battery_mv[i] = batteryPresent ? clampUint16(batteryV[i] * 1000.0f) : 0;
    packet.current_ma[i] = inaReady[i] ? clampInt16(currentmA[i]) : 0;
    packet.temp_centi_c[i] = tempValid[i] ? clampInt16(curTemp[i] * 100.0f) : 0;
    packet.energy_mah[i] = clampUint16(mah[i]);
    batteryLow = batteryLow || (batteryPresent && percent <= 15);
  }
  packet.header.battery_low = batteryLow ? 1 : 0;

  esp_err_t result = esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                                  reinterpret_cast<const uint8_t *>(&packet),
                                  sizeof(packet));
  if (result == ESP_OK) {
    espNowSendOkCount++;
  } else {
    espNowSendFailCount++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[CHG] ESP32-S3 smart charging dock booting");
  initializeEspNow();

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    pinMode(GATE_PINS[i], OUTPUT);
    digitalWrite(GATE_PINS[i], GATE_OFF_LEVEL);
  }

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastClk = digitalRead(ENC_CLK);
  lastButtonState = digitalRead(ENC_SW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  displayReady = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (displayReady) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("SMART CHG DOCK");
    display.println("Booting...");
    display.display();
  } else {
    Serial.println("[CHG] OLED not found at 0x3C");
  }

  temps.begin();
  temps.setResolution(10);

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    inaReady[i] = sensors[i]->begin(&Wire);
    Serial.print("[CHG] INA");
    Serial.print(i + 1);
    Serial.println(inaReady[i] ? " ready" : " not found");
  }

  allGatesOff();
  lastTickMs = millis();
  lastSensorMs = 0;
  lastDisplayMs = 0;
  lastStatusMs = 0;
  lastReportMs = 0;
  Serial.println("[CHG] Setup complete");
}

void loop() {
  uint32_t now = millis();

  handleEncoder();

  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    updateSensorsAndChargeLogic();
  }

  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    updateDisplay();
  }

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printStatus();
  }

  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    sendChargingDockStatus();
  }
}
