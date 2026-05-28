/*
 * AirTrixx Wireless Audio Dock ESP32-S3 Firmware
 * 
 * Flow:
 *  1. Boots, initializes custom I2C pins for 16x2 LCD, setups WiFi STA and ESP-NOW.
 *  2. Registers ANTENNA_MAC_PLACEHOLDER as a peer.
 *  3. Enters "Wait for Clap" mode continuously.
 *  4. Edge Impulse model captures and classifies audio.
 *  5. Once a single or double clap is detected, it uninstalls the 16-bit Edge Impulse I2S task.
 *  6. Sends MSG_AUDIODOCK_DATA packet to Antenna with clap trigger type and audio_size (96044).
 *  7. Initializes 32-bit audio recording at 16kHz for 3 seconds (96,044 bytes total).
 *  8. Saves WAV data into internal RAM.
 *  9. Uninstalls recording I2S driver.
 *  10. Slices the WAV buffer into 200-byte chunks and sends them over ESP-NOW using MSG_AUDIODOCK_AUDIO_CHUNK.
 *  11. Enters "Waiting for Transcript" state, listening for MSG_AUDIODOCK_TRANSCRIPT over ESP-NOW.
 *  12. When transcript is received, renders it beautifully on the 16x2 LCD.
 *  13. Returns to "Wait for Clap" mode.
 */

#define EIDSP_QUANTIZE_FILTERBANK   0
#include <ESP32-S3-Clap-Sensor_inferencing.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
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

// Match hardware wiring
#define I2S_WS 7
#define I2S_SD 17
#define I2S_SCK 4
#define I2S_PORT I2S_NUM_0

#define SAMPLE_RATE 16000
#define RECORD_SECONDS 3
#define RECORD_GAIN 32
#define WAV_HEADER_BYTES 44
#define AUDIO_DATA_BYTES (RECORD_SECONDS * SAMPLE_RATE * sizeof(int16_t))
#define AUDIO_TOTAL_BYTES (WAV_HEADER_BYTES + AUDIO_DATA_BYTES)
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.

// -------------------- LCD Setup --------------------
#define I2C_SDA 41
#define I2C_SCL 42
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool i2sReady = false;
uint8_t *audioBuffer = nullptr;
uint16_t audioDockSequence = 0;

// -------------------- Edge Impulse Inferencing Variables --------------------
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;
static volatile bool taskRunning = false;

// State machine states
enum AudioDockState {
  STATE_INIT,
  STATE_WAIT_CLAP,
  STATE_RECORDING,
  STATE_STREAMING,
  STATE_WAIT_TRANSCRIPT
};
static AudioDockState currentState = STATE_INIT;
static String lastTranscriptText = "";
static bool transcriptReceived = false;
static uint32_t transcriptWaitStartMs = 0;
static const uint32_t TRANSCRIPT_TIMEOUT_MS = 30000;

// -------------------- WAV Header & Recording Helper Functions --------------------
void writeLE16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

void writeLE32(uint8_t *buffer, size_t offset, uint32_t value) {
  buffer[offset] = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
  buffer[offset + 3] = (uint8_t)((value >> 24) & 0xFF);
}

void writeWavHeader(uint8_t *buffer, uint32_t dataBytes) {
  const uint16_t channels = 1;
  const uint16_t bitsPerSample = 16;
  const uint32_t byteRate = SAMPLE_RATE * channels * (bitsPerSample / 8);
  const uint16_t blockAlign = channels * (bitsPerSample / 8);

  memcpy(buffer + 0, "RIFF", 4);
  writeLE32(buffer, 4, dataBytes + 36);
  memcpy(buffer + 8, "WAVE", 4);
  memcpy(buffer + 12, "fmt ", 4);
  writeLE32(buffer, 16, 16);
  writeLE16(buffer, 20, 1);
  writeLE16(buffer, 22, channels);
  writeLE32(buffer, 24, SAMPLE_RATE);
  writeLE32(buffer, 28, byteRate);
  writeLE16(buffer, 32, blockAlign);
  writeLE16(buffer, 34, bitsPerSample);
  memcpy(buffer + 36, "data", 4);
  writeLE32(buffer, 40, dataBytes);
}

void flushI2S() {
  int32_t discard[256];
  size_t bytesRead = 0;
  for (int i = 0; i < 6; i++) {
    i2s_read(I2S_PORT, discard, sizeof(discard), &bytesRead, 20 / portTICK_PERIOD_MS);
  }
}

bool initI2SMic() {
  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2sConfig, 0, NULL);
  if (err != ESP_OK) {
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pinConfig);
  if (err != ESP_OK) {
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  i2sReady = true;
  return true;
}

bool allocateAudioBuffer() {
  if (audioBuffer != nullptr) {
    return true;
  }
  audioBuffer = (uint8_t *)malloc(AUDIO_TOTAL_BYTES);
  return (audioBuffer != nullptr);
}

bool recordWavToMemory(uint32_t *wavBytes) {
  if (!i2sReady || !allocateAudioBuffer()) {
    return false;
  }

  writeWavHeader(audioBuffer, AUDIO_DATA_BYTES);
  flushI2S();

  uint32_t samplesWritten = 0;
  const uint32_t totalSamples = RECORD_SECONDS * SAMPLE_RATE;
  int32_t i2sBuffer[512];
  int16_t pcmBuffer[512];

  while (samplesWritten < totalSamples) {
    uint32_t samplesToRead = totalSamples - samplesWritten;
    if (samplesToRead > 512) samplesToRead = 512;

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, i2sBuffer, samplesToRead * sizeof(int32_t), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0) {
      return false;
    }

    size_t samples = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < samples; i++) {
      int32_t sample16 = i2sBuffer[i] >> 16;
      int32_t boosted = sample16 * RECORD_GAIN;
      if (boosted > 32767) boosted = 32767;
      if (boosted < -32768) boosted = -32768;
      pcmBuffer[i] = (int16_t)boosted;
    }

    memcpy(audioBuffer + WAV_HEADER_BYTES + (samplesWritten * sizeof(int16_t)),
           pcmBuffer,
           samples * sizeof(int16_t));
    samplesWritten += samples;
  }

  uint32_t dataBytes = samplesWritten * sizeof(int16_t);
  writeWavHeader(audioBuffer, dataBytes);
  *wavBytes = dataBytes + WAV_HEADER_BYTES;
  return true;
}

// -------------------- LCD UI Display Helper Functions --------------------
void displayLCDStatus(const String &line1, const String &line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void displayLCDTranscript(const String &transcript) {
  lcd.clear();
  String clean = transcript;
  clean.trim();
  if (clean.length() == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Transcript:");
    lcd.setCursor(0, 1);
    lcd.print("(empty)");
    return;
  }

  if (clean.length() <= 16) {
    lcd.setCursor(0, 0);
    lcd.print("Transcript:");
    lcd.setCursor(0, 1);
    lcd.print(clean);
  } else {
    lcd.setCursor(0, 0);
    lcd.print(clean.substring(0, min((int)clean.length(), 16)));
    lcd.setCursor(0, 1);
    lcd.print(clean.substring(16, min((int)clean.length(), 32)));
  }
}

// -------------------- Edge Impulse Classifier Helper Functions --------------------
static void audio_inference_callback(uint32_t n_bytes) {
  for (int i = 0; i < n_bytes >> 1; i++) {
    inference.buffer[inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static void capture_samples(void* arg) {
  taskRunning = true;
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {
    i2s_read(I2S_PORT, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read <= 0) {
      delay(10);
    } else {
      for (int x = 0; x < i2s_bytes_to_read / 2; x++) {
        sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
      }

      if (record_status) {
        audio_inference_callback(i2s_bytes_to_read);
      } else {
        break;
      }
    }
  }
  taskRunning = false;
  vTaskDelete(NULL);
}

static int ei_i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = sampling_rate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD
  };

  esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (ret != ESP_OK) return int(ret);

  ret = i2s_set_pin(I2S_PORT, &pin_config);
  if (ret != ESP_OK) return int(ret);

  ret = i2s_zero_dma_buffer(I2S_PORT);
  return int(ret);
}

static int ei_i2s_deinit(void) {
  i2s_driver_uninstall(I2S_PORT);
  i2sReady = false;
  return 0;
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
  if (inference.buffer == NULL) {
    return false;
  }

  inference.buf_count  = 0;
  inference.n_samples  = n_samples;
  inference.buf_ready  = 0;

  ei_i2s_init(EI_CLASSIFIER_FREQUENCY);
  delay(100);
  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
  return true;
}

static bool microphone_inference_record(void) {
  while (inference.buf_ready == 0) {
    delay(10);
  }
  inference.buf_ready = 0;
  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
  return 0;
}

static void microphone_inference_end(void) {
  record_status = false;
  
  // Wait for the capture task to exit cleanly
  uint32_t timeout = millis() + 500;
  while (taskRunning && millis() < timeout) {
    delay(5);
  }
  delay(50); // Additional safety margin
  
  ei_i2s_deinit();
  free(inference.buffer);
}

// -------------------- ESP-NOW Handlers & Config --------------------
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
  return (esp_now_add_peer(&peer) == ESP_OK);
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

  if (header.msg_type == MSG_AUDIODOCK_TRANSCRIPT && len == static_cast<int>(sizeof(AudioDockTranscriptPacket))) {
    AudioDockTranscriptPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    lastTranscriptText = String(packet.transcript);
    transcriptReceived = true;
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

// -------------------- Main Arduino Setup & Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize LCD Screen
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AirTrixx Audio");
  lcd.setCursor(0, 1);
  lcd.print("Dock Booting...");
  delay(1000);

  // Configure Wi-Fi STA and ESP-NOW
  configureWiFiChannel();

  // Read and display MAC address
  uint8_t mac[6] = {};
  WiFi.macAddress(mac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("AUDIO_DOCK_MAC: ");
  Serial.println(macStr);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAC Address:");
  lcd.setCursor(0, 1);
  lcd.print(macStr);
  delay(4000); // Display for 4 seconds so the user can easily see it!

  if (esp_now_init() != ESP_OK) {
    displayLCDStatus("WiFi Error", "ESP-NOW Failed");
    while (1) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);
  
  // Register PC Antenna peer
  addEspNowPeer(ANTENNA_MAC_PLACEHOLDER);

  displayLCDStatus("System Ready", "Clap to speak!");
  currentState = STATE_WAIT_CLAP;
}

void loop() {
  switch (currentState) {
    case STATE_WAIT_CLAP: {
      uint8_t mac[6] = {};
      WiFi.macAddress(mac);
      char macLcd[18];
      snprintf(macLcd, sizeof(macLcd), "MAC:%02X%02X%02X%02X%02X%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      displayLCDStatus("Clap to speak!", macLcd);
      
      if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
        displayLCDStatus("Inference Err", "Memory failed");
        delay(3000);
        break;
      }

      bool clapDetected = false;
      uint8_t triggerType = 0; // 1 = Single, 2 = Double

      while (!clapDetected) {
        bool m = microphone_inference_record();
        if (!m) {
          continue;
        }

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
          continue;
        }

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          String label = String(result.classification[ix].label);
          float val = result.classification[ix].value;
          if (val >= 0.70f) {
            if (label.equalsIgnoreCase("Double clap")) {
              triggerType = 2;
              clapDetected = true;
              break;
            } else if (label.equalsIgnoreCase("Single clap")) {
              triggerType = 1;
              clapDetected = true;
              break;
            }
          }
        }
      }

      // Clap detected! Stop model classification
      microphone_inference_end();
      displayLCDStatus("Clap Detected!", "Speak now...");
      delay(600); // Small pause for user preparation

      // Send trigger packet to Antenna over ESP-NOW
      AudioDockDataPacket dataPacket = {};
      fillHeader(dataPacket.header,
                 MSG_AUDIODOCK_DATA,
                 DEVICE_AUDIODOCK,
                 ++audioDockSequence,
                 millis(),
                 false);
      dataPacket.clap_detected = 1;
      dataPacket.clap_type = triggerType;
      dataPacket.audio_size = AUDIO_TOTAL_BYTES;

      esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                   reinterpret_cast<const uint8_t *>(&dataPacket),
                   sizeof(dataPacket));
      delay(50); // Let the packet reach

      currentState = STATE_RECORDING;
      break;
    }

    case STATE_RECORDING: {
      displayLCDStatus("Status: Record", "Listening...");
      
      if (!initI2SMic()) {
        displayLCDStatus("Mic Error", "Failed to start");
        delay(2000);
        currentState = STATE_WAIT_CLAP;
        break;
      }

      uint32_t wavSize = 0;
      bool recordedOk = recordWavToMemory(&wavSize);
      
      // Uninstall microphone I2S driver
      i2s_driver_uninstall(I2S_PORT);
      i2sReady = false;

      if (!recordedOk) {
        displayLCDStatus("Record Error", "Failed saving");
        delay(2000);
        currentState = STATE_WAIT_CLAP;
        break;
      }

      currentState = STATE_STREAMING;
      break;
    }

    case STATE_STREAMING: {
      displayLCDStatus("Status: Sending", "Uploading...");

      // Send WAV in 200-byte packets over ESP-NOW
      uint32_t sentBytes = 0;
      uint32_t chunkIndex = 0;

      while (sentBytes < AUDIO_TOTAL_BYTES) {
        AudioDockChunkPacket chunkPacket = {};
        fillHeader(chunkPacket.header,
                   MSG_AUDIODOCK_AUDIO_CHUNK,
                   DEVICE_AUDIODOCK,
                   ++audioDockSequence,
                   millis(),
                   false);
        chunkPacket.chunk_index = chunkIndex++;
        
        uint32_t remaining = AUDIO_TOTAL_BYTES - sentBytes;
        uint16_t currentChunkLen = (remaining > 200) ? 200 : remaining;
        chunkPacket.chunk_len = currentChunkLen;
        memcpy(chunkPacket.data, audioBuffer + sentBytes, currentChunkLen);

        esp_err_t result = esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                                        reinterpret_cast<const uint8_t *>(&chunkPacket),
                                        sizeof(chunkPacket));
        
        int retries = 0;
        while (result != ESP_OK && retries < 10) {
          delay(5);
          result = esp_now_send(ANTENNA_MAC_PLACEHOLDER,
                                reinterpret_cast<const uint8_t *>(&chunkPacket),
                                sizeof(chunkPacket));
          retries++;
        }
        
        sentBytes += currentChunkLen;
        
        // Minor delay to prevent peer queue overflow (crucial for ESP-NOW)
        delay(20);
      }

      transcriptReceived = false;
      transcriptWaitStartMs = millis();
      currentState = STATE_WAIT_TRANSCRIPT;
      break;
    }

    case STATE_WAIT_TRANSCRIPT: {
      displayLCDStatus("Status: Wait...", "Transcribing...");
      
      while (!transcriptReceived && (millis() - transcriptWaitStartMs < TRANSCRIPT_TIMEOUT_MS)) {
        delay(50);
      }

      if (transcriptReceived) {
        displayLCDTranscript(lastTranscriptText);
        delay(5000); // Display the transcript for 5 seconds
      } else {
        displayLCDStatus("Status: Timeout", "No response");
        delay(3000);
      }

      currentState = STATE_WAIT_CLAP;
      break;
    }
  }
}
