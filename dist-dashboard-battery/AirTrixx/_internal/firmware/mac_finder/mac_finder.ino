#include <Arduino.h>
#include <WiFi.h>

void printMacArray(const uint8_t mac[6]) {
  Serial.print("uint8_t DEVICE_MAC[] = {");
  for (int i = 0; i < 6; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print("0x");
    if (mac[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
  Serial.println("};");
}

void setup() {
  Serial.begin(115200);
  delay(800);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  uint8_t mac[6];
  WiFi.macAddress(mac);

  Serial.println();
  Serial.println("AirTrixx ESP32 MAC Finder");
  Serial.print("Chip model: ");
  Serial.println(ESP.getChipModel());
  Serial.print("Chip revision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("WiFi STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Copy-paste format:");
  printMacArray(mac);
}

void loop() {
  delay(2000);
}

