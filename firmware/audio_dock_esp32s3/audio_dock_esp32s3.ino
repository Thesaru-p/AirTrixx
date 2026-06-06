/*
 * AirTrixx Wireless Audio Dock ESP32-S3 Firmware
 * 
 * Flow:
 *  1. Boots, initializes custom I2C pins for 16x2 LCD, setups WiFi STA and ESP-NOW.
 *  2. Registers ANTENNA_MAC_PLACEHOLDER as a peer.
 *  3. Enters "Wait for Clap" mode continuously.
 *  4. Edge Impulse model captures and classifies audio.
 *  5. Once a single or double clap is detected, it uninstalls the 32-bit Edge Impulse I2S task.
 *  6. Initializes 32-bit audio recording at 16kHz for 3 seconds (96,044 bytes total).
 *  7. Sends MSG_AUDIODOCK_DATA packet to Antenna with clap trigger type and audio_size.
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
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <math.h>

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
#define SPEAK_DELAY_MS 900
#define CLAP_INFERENCE_GAIN 16
#define CLAP_LABEL_THRESHOLD 0.35f
#define CLAP_COMBINED_THRESHOLD 0.45f
#define PRINT_CLAP_SCORES true
#define WAV_HEADER_BYTES 44
#define AUDIO_DATA_BYTES (RECORD_SECONDS * SAMPLE_RATE * sizeof(int16_t))
#define AUDIO_TOTAL_BYTES (WAV_HEADER_BYTES + AUDIO_DATA_BYTES)
static const int8_t WIFI_TX_POWER_QDBM = 34;  // 8.5 dBm in 0.25 dBm units.

// -------------------- SD card pins --------------------
#define SD_CS_PIN 47
#define SD_SCK_PIN 38
#define SD_MISO_PIN 40
#define SD_MOSI_PIN 39
#define SD_SPI_HZ 4000000
static const char *AUDIO_WAV_PATH = "/AIRDCK.WAV";

// -------------------- MAX98357A I2S speaker --------------------
#define SPEAKER_BCLK_PIN 21
#define SPEAKER_LRC_PIN 48
#define SPEAKER_DIN_PIN 1
#define SPEAKER_I2S_PORT I2S_NUM_1
#define SPEAKER_SAMPLE_RATE 22050
#define SPEAKER_VOLUME 20000

// -------------------- LCD Setup --------------------
#define I2C_SDA 41
#define I2C_SCL 42
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- WS2812 LED ring --------------------
#define LED_RING_PIN 15
#define LED_RING_COUNT 16
#define LED_RING_BRIGHTNESS 32
Adafruit_NeoPixel ledRing(LED_RING_COUNT, LED_RING_PIN, NEO_GRB + NEO_KHZ800);

bool i2sReady = false;
bool speakerReady = false;
bool sdReady = false;
uint8_t *audioBuffer = nullptr;
uint16_t audioDockSequence = 0;
uint8_t pendingTriggerType = 0;
uint32_t recordedWavBytes = 0;
String lastRecordError = "";

// -------------------- Edge Impulse Inferencing Variables --------------------
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static int16_t sampleBuffer[sample_buffer_size];
static int32_t inferenceRawBuffer[sample_buffer_size];
static uint32_t inferenceWindowPeak = 0;
static volatile uint32_t lastInferencePeak = 0;
static bool debug_nn = false;
static bool record_status = true;
static volatile bool taskRunning = false;

void displayLCDStatus(const String &line1, const String &line2);
void displayLCDTranscript(const String &transcript);
void ringChase(uint8_t r, uint8_t g, uint8_t b, uint16_t head);
void ringProgress(uint32_t current, uint32_t total, uint8_t r, uint8_t g, uint8_t b);
void ringFlash(uint8_t r, uint8_t g, uint8_t b, uint8_t flashes, uint16_t onMs, uint16_t offMs);
void ringShowReady();
void ringShowSuccess();
void ringShowError();
void runLedRingSelfTest();
void disableOnboardLed();
void playTranscriptionDoneSound();
void runSpeakerSelfTest();

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
static String lastRemoteCommandText = "";
static uint32_t lastRemoteCommandMs = 0;
static bool transcriptReceived = false;
static uint32_t transcriptWaitStartMs = 0;
static const uint32_t TRANSCRIPT_TIMEOUT_MS = 30000;

// -------------------- LED Ring Helper Functions --------------------
void ringClear() {
  ledRing.clear();
  ledRing.show();
}

void ringFill(uint8_t r, uint8_t g, uint8_t b) {
  ledRing.fill(ledRing.Color(r, g, b));
  ledRing.show();
}

void ringSetScaledPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b, uint8_t scale) {
  ledRing.setPixelColor(pixel,
                        ((uint16_t)r * scale) / 255,
                        ((uint16_t)g * scale) / 255,
                        ((uint16_t)b * scale) / 255);
}

void ringChase(uint8_t r, uint8_t g, uint8_t b, uint16_t head) {
  ledRing.clear();
  for (uint8_t tail = 0; tail < 4; tail++) {
    uint16_t pixel = (head + LED_RING_COUNT - tail) % LED_RING_COUNT;
    uint8_t scale = 255 / (tail + 1);
    ringSetScaledPixel(pixel, r, g, b, scale);
  }
  ledRing.show();
}

void ringProgress(uint32_t current, uint32_t total, uint8_t r, uint8_t g, uint8_t b) {
  uint16_t litPixels = 0;
  if (total > 0) {
    litPixels = (uint16_t)(((uint64_t)current * LED_RING_COUNT + total - 1) / total);
  }
  if (litPixels > LED_RING_COUNT) {
    litPixels = LED_RING_COUNT;
  }

  ledRing.clear();
  for (uint16_t i = 0; i < litPixels; i++) {
    ledRing.setPixelColor(i, ledRing.Color(r, g, b));
  }
  ledRing.show();
}

void ringFlash(uint8_t r, uint8_t g, uint8_t b, uint8_t flashes, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < flashes; i++) {
    ringFill(r, g, b);
    delay(onMs);
    ringClear();
    delay(offMs);
  }
}

void ringShowReady() {
  ringFill(0, 0, 24);
}

void ringShowSuccess() {
  ringFill(0, 48, 0);
}

void ringShowError() {
  ringFill(64, 0, 0);
}

void runLedRingSelfTest() {
  Serial.printf("LED_RING_TEST_BEGIN pin=%u count=%u\n", LED_RING_PIN, LED_RING_COUNT);
  displayLCDStatus("LED Ring Test", "GPIO 15");

  ringFlash(64, 0, 0, 1, 250, 100);
  ringFlash(0, 64, 0, 1, 250, 100);
  ringFlash(0, 0, 64, 1, 250, 100);

  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    ledRing.clear();
    ledRing.setPixelColor(i, ledRing.Color(48, 48, 48));
    ledRing.show();
    delay(70);
  }

  ringShowReady();
  displayLCDStatus("LED Ring Test", "Done");
  Serial.println("LED_RING_TEST_DONE");
}

void disableOnboardLed() {
#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  pinMode(RGB_BUILTIN, OUTPUT);
  digitalWrite(RGB_BUILTIN, LOW);
#elif defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

// -------------------- MAX98357A Speaker Helper Functions --------------------
bool initSpeakerI2S() {
  if (speakerReady) {
    return true;
  }

  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SPEAKER_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = SPEAKER_BCLK_PIN,
    .ws_io_num = SPEAKER_LRC_PIN,
    .data_out_num = SPEAKER_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(SPEAKER_I2S_PORT, &i2sConfig, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Speaker I2S driver install failed: %d\n", err);
    return false;
  }

  err = i2s_set_pin(SPEAKER_I2S_PORT, &pinConfig);
  if (err != ESP_OK) {
    Serial.printf("Speaker I2S pin setup failed: %d\n", err);
    i2s_driver_uninstall(SPEAKER_I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer(SPEAKER_I2S_PORT);
  speakerReady = true;
  Serial.printf("Speaker ready: BCLK=%u LRC=%u DIN=%u\n",
                SPEAKER_BCLK_PIN,
                SPEAKER_LRC_PIN,
                SPEAKER_DIN_PIN);
  return true;
}

void deinitSpeakerI2S() {
  if (!speakerReady) {
    disableOnboardLed();
    return;
  }

  i2s_zero_dma_buffer(SPEAKER_I2S_PORT);
  i2s_driver_uninstall(SPEAKER_I2S_PORT);
  speakerReady = false;
  disableOnboardLed();
}

void playSpeakerSilence(uint16_t durationMs) {
  if (!speakerReady && !initSpeakerI2S()) {
    return;
  }

  const uint16_t framesPerBuffer = 128;
  int16_t samples[framesPerBuffer * 2] = {0};
  uint32_t totalFrames = ((uint32_t)SPEAKER_SAMPLE_RATE * durationMs) / 1000;

  while (totalFrames > 0) {
    uint32_t framesThis = totalFrames > framesPerBuffer ? framesPerBuffer : totalFrames;
    size_t bytesWritten = 0;
    i2s_write(SPEAKER_I2S_PORT,
              samples,
              framesThis * 2 * sizeof(int16_t),
              &bytesWritten,
              portMAX_DELAY);
    totalFrames -= framesThis;
  }
}

void playSpeakerTone(uint16_t frequency, uint16_t durationMs) {
  if (!speakerReady && !initSpeakerI2S()) {
    return;
  }

  const float twoPi = 6.28318530718f;
  const uint16_t framesPerBuffer = 128;
  int16_t samples[framesPerBuffer * 2];
  uint32_t totalFrames = ((uint32_t)SPEAKER_SAMPLE_RATE * durationMs) / 1000;
  float phase = 0.0f;
  float phaseStep = twoPi * (float)frequency / (float)SPEAKER_SAMPLE_RATE;

  while (totalFrames > 0) {
    uint32_t framesThis = totalFrames > framesPerBuffer ? framesPerBuffer : totalFrames;

    for (uint32_t i = 0; i < framesThis; i++) {
      int16_t sample = (int16_t)(sinf(phase) * SPEAKER_VOLUME);
      samples[(i * 2) + 0] = sample;
      samples[(i * 2) + 1] = sample;
      phase += phaseStep;
      if (phase >= twoPi) {
        phase -= twoPi;
      }
    }

    size_t bytesWritten = 0;
    i2s_write(SPEAKER_I2S_PORT,
              samples,
              framesThis * 2 * sizeof(int16_t),
              &bytesWritten,
              portMAX_DELAY);
    totalFrames -= framesThis;
  }
}

void playTranscriptionDoneSound() {
  playSpeakerTone(880, 120);
  playSpeakerSilence(50);
  playSpeakerTone(1175, 180);
  playSpeakerSilence(30);
  deinitSpeakerI2S();
}

void runSpeakerSelfTest() {
  Serial.printf("SPEAKER_TEST_BEGIN BCLK=%u LRC=%u DIN=%u\n",
                SPEAKER_BCLK_PIN,
                SPEAKER_LRC_PIN,
                SPEAKER_DIN_PIN);
  displayLCDStatus("Speaker Test", "MAX98357A");
  playTranscriptionDoneSound();
  displayLCDStatus("Speaker Test", "Done");
  Serial.println("SPEAKER_TEST_DONE");
}

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

  Serial.printf("Audio RAM request: %u bytes, free=%u, largest=%u\n",
                (unsigned int)AUDIO_TOTAL_BYTES,
                (unsigned int)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  audioBuffer = (uint8_t *)heap_caps_malloc(AUDIO_TOTAL_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (audioBuffer == nullptr) {
    audioBuffer = (uint8_t *)heap_caps_malloc(AUDIO_TOTAL_BYTES, MALLOC_CAP_8BIT);
  }

  if (audioBuffer == nullptr) {
    lastRecordError = "RAM alloc failed";
    Serial.printf("Audio RAM allocation failed: need=%u free=%u largest=%u\n",
                  (unsigned int)AUDIO_TOTAL_BYTES,
                  (unsigned int)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                  (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return false;
  }
  Serial.printf("Audio RAM ready: %u bytes\n", (unsigned int)AUDIO_TOTAL_BYTES);
  return (audioBuffer != nullptr);
}

bool initSDCard() {
  if (sdReady) {
    SD.end();
    SPI.end();
    sdReady = false;
    delay(20);
  }

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(10);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  Serial.printf("SD init begin: CS=%u SCK=%u MISO=%u MOSI=%u hz=%u\n",
                SD_CS_PIN,
                SD_SCK_PIN,
                SD_MISO_PIN,
                SD_MOSI_PIN,
                SD_SPI_HZ);

  if (!SD.begin(SD_CS_PIN, SPI, SD_SPI_HZ)) {
    Serial.println("SD init failed. Check CS/SCK/MISO/MOSI pins and card format.");
    sdReady = false;
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card detected.");
    sdReady = false;
    return false;
  }

  Serial.print("SD card ready. Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC/SDXC");
  else Serial.println("Unknown");

  sdReady = true;
  return true;
}

bool recordWavToMemory(uint32_t *wavBytes) {
  lastRecordError = "";
  if (!i2sReady) {
    lastRecordError = "I2S not ready";
    Serial.println("Cannot record: I2S mic is not ready.");
    return false;
  }

  if (!allocateAudioBuffer()) {
    return false;
  }

  writeWavHeader(audioBuffer, AUDIO_DATA_BYTES);
  flushI2S();

  Serial.printf("RECORDING_START seconds=%u\n", (unsigned int)RECORD_SECONDS);
  uint32_t startedAt = millis();
  uint32_t samplesWritten = 0;
  const uint32_t totalSamples = RECORD_SECONDS * SAMPLE_RATE;
  uint32_t lastRingUpdate = 0;
  int32_t i2sBuffer[512];
  int16_t pcmBuffer[512];
  ringProgress(0, totalSamples, 80, 24, 0);

  while (samplesWritten < totalSamples) {
    uint32_t samplesToRead = totalSamples - samplesWritten;
    if (samplesToRead > 512) samplesToRead = 512;

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, i2sBuffer, samplesToRead * sizeof(int32_t), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0) {
      lastRecordError = "I2S read failed";
      Serial.printf("I2S recording read failed: err=%d bytes=%u at_sample=%u\n",
                    err,
                    (unsigned int)bytesRead,
                    (unsigned int)samplesWritten);
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

    if (millis() - lastRingUpdate >= 120 || samplesWritten >= totalSamples) {
      ringProgress(samplesWritten, totalSamples, 80, 24, 0);
      lastRingUpdate = millis();
    }
  }

  uint32_t dataBytes = samplesWritten * sizeof(int16_t);
  writeWavHeader(audioBuffer, dataBytes);
  *wavBytes = dataBytes + WAV_HEADER_BYTES;
  Serial.printf("RECORDING_DONE ms=%lu wav_bytes=%u\n",
                (unsigned long)(millis() - startedAt),
                (unsigned int)*wavBytes);
  return true;
}

bool recordWavToSD(uint32_t *wavBytes) {
  lastRecordError = "";
  if (!initSDCard()) {
    lastRecordError = "SD not ready";
    return false;
  }
  if (!i2sReady) {
    lastRecordError = "I2S not ready";
    Serial.println("Cannot record: I2S mic is not ready.");
    return false;
  }

  if (SD.exists(AUDIO_WAV_PATH)) {
    SD.remove(AUDIO_WAV_PATH);
  }

  File file = SD.open(AUDIO_WAV_PATH, FILE_WRITE);
  if (!file) {
    lastRecordError = "SD open failed";
    Serial.println("Failed to open SD WAV file for writing.");
    return false;
  }

  uint8_t header[WAV_HEADER_BYTES];
  writeWavHeader(header, AUDIO_DATA_BYTES);
  if (file.write(header, WAV_HEADER_BYTES) != WAV_HEADER_BYTES) {
    lastRecordError = "SD write failed";
    Serial.println("Failed to write WAV header to SD.");
    file.close();
    return false;
  }

  displayLCDStatus("Get ready", "Recording soon");
  ringFlash(0, 64, 0, 2, 120, 80);
  delay(SPEAK_DELAY_MS);

  flushI2S();
  displayLCDStatus("Recording...", "Speak now!");
  Serial.printf("SD_RECORD_START seconds=%u path=%s\n",
                (unsigned int)RECORD_SECONDS,
                AUDIO_WAV_PATH);

  uint32_t startedAt = millis();
  uint32_t samplesWritten = 0;
  const uint32_t totalSamples = RECORD_SECONDS * SAMPLE_RATE;
  uint32_t lastRingUpdate = 0;
  int32_t i2sBuffer[512];
  int16_t pcmBuffer[512];
  ringProgress(0, totalSamples, 80, 24, 0);

  while (samplesWritten < totalSamples) {
    uint32_t samplesToRead = totalSamples - samplesWritten;
    if (samplesToRead > 512) samplesToRead = 512;

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT,
                             i2sBuffer,
                             samplesToRead * sizeof(int32_t),
                             &bytesRead,
                             portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0) {
      lastRecordError = "I2S read failed";
      Serial.printf("I2S SD record read failed: err=%d bytes=%u at_sample=%u\n",
                    err,
                    (unsigned int)bytesRead,
                    (unsigned int)samplesWritten);
      file.close();
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

    size_t pcmBytes = samples * sizeof(int16_t);
    if (file.write(reinterpret_cast<const uint8_t *>(pcmBuffer), pcmBytes) != pcmBytes) {
      lastRecordError = "SD write failed";
      Serial.println("Failed while writing PCM data to SD.");
      file.close();
      return false;
    }

    samplesWritten += samples;
    if (millis() - lastRingUpdate >= 120 || samplesWritten >= totalSamples) {
      ringProgress(samplesWritten, totalSamples, 80, 24, 0);
      lastRingUpdate = millis();
    }
  }

  uint32_t dataBytes = samplesWritten * sizeof(int16_t);
  writeWavHeader(header, dataBytes);
  file.seek(0);
  file.write(header, WAV_HEADER_BYTES);
  file.close();

  *wavBytes = dataBytes + WAV_HEADER_BYTES;
  Serial.printf("SD_RECORD_DONE ms=%lu wav_bytes=%u path=%s\n",
                (unsigned long)(millis() - startedAt),
                (unsigned int)*wavBytes,
                AUDIO_WAV_PATH);
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
static void audio_inference_callback(uint32_t samplesRead) {
  for (uint32_t i = 0; i < samplesRead; i++) {
    int32_t value = sampleBuffer[i];
    uint32_t magnitude = value < 0 ? (uint32_t)(-value) : (uint32_t)value;
    if (magnitude > inferenceWindowPeak) {
      inferenceWindowPeak = magnitude;
    }

    inference.buffer[inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      lastInferencePeak = inferenceWindowPeak;
      inferenceWindowPeak = 0;
      inference.buf_count = 0;
      inference.buf_ready = 1;
      break;
    }
  }
}

static void capture_samples(void* arg) {
  taskRunning = true;
  const uint32_t samplesToRead = (uint32_t)arg;

  while (record_status) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_PORT,
                             (void*)inferenceRawBuffer,
                             samplesToRead * sizeof(int32_t),
                             &bytes_read,
                             100 / portTICK_PERIOD_MS);

    if (err != ESP_OK || bytes_read <= 0) {
      delay(10);
    } else {
      uint32_t samplesRead = bytes_read / sizeof(int32_t);
      if (samplesRead > sample_buffer_size) {
        samplesRead = sample_buffer_size;
      }

      for (uint32_t x = 0; x < samplesRead; x++) {
        int32_t sample16 = inferenceRawBuffer[x] >> 16;
        int32_t boosted = sample16 * CLAP_INFERENCE_GAIN;
        if (boosted > 32767) boosted = 32767;
        if (boosted < -32768) boosted = -32768;
        sampleBuffer[x] = (int16_t)boosted;
      }

      if (record_status && inference.buf_ready == 0) {
        audio_inference_callback(samplesRead);
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

  if (ei_i2s_init(EI_CLASSIFIER_FREQUENCY) != 0) {
    Serial.println("Failed to start I2S for Edge Impulse!");
    free(inference.buffer);
    inference.buffer = nullptr;
    return false;
  }

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
  if (inference.buffer != nullptr) {
    free(inference.buffer);
    inference.buffer = nullptr;
  }
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

    String incoming = String(packet.transcript);
    incoming.trim();
    if (incoming == "__CMD:LEDTEST__") {
      if (lastRemoteCommandText == incoming && millis() - lastRemoteCommandMs < 1500) {
        return;
      }
      lastRemoteCommandText = incoming;
      lastRemoteCommandMs = millis();
      runLedRingSelfTest();
      displayLCDStatus("System Ready", "Clap to speak!");
      return;
    }
    if (incoming == "__CMD:SPEAKERTEST__") {
      if (lastRemoteCommandText == incoming && millis() - lastRemoteCommandMs < 1500) {
        return;
      }
      lastRemoteCommandText = incoming;
      lastRemoteCommandMs = millis();
      runSpeakerSelfTest();
      displayLCDStatus("System Ready", "Clap to speak!");
      return;
    }

    lastTranscriptText = incoming;
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

bool sendEspNowToAntenna(const uint8_t *data, size_t len, uint8_t retries = 10) {
  esp_err_t result = ESP_FAIL;
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    result = esp_now_send(ESPNOW_BROADCAST_MAC, data, len);
    if (result == ESP_OK) {
      return true;
    }
    delay(5);
  }
  Serial.printf("ESP-NOW send failed: %d\n", result);
  return false;
}

bool sendAudioDockTrigger(uint8_t triggerType, uint32_t audioSize) {
  AudioDockDataPacket dataPacket = {};
  fillHeader(dataPacket.header,
             MSG_AUDIODOCK_DATA,
             DEVICE_AUDIODOCK,
             ++audioDockSequence,
             millis(),
             false);
  dataPacket.clap_detected = 1;
  dataPacket.clap_type = triggerType;
  dataPacket.audio_size = audioSize;

  return sendEspNowToAntenna(reinterpret_cast<const uint8_t *>(&dataPacket), sizeof(dataPacket), 20);
}

bool sendAudioDockChunk(const uint8_t *data, uint16_t len, uint32_t chunkIndex) {
  AudioDockChunkPacket chunkPacket = {};
  fillHeader(chunkPacket.header,
             MSG_AUDIODOCK_AUDIO_CHUNK,
             DEVICE_AUDIODOCK,
             ++audioDockSequence,
             millis(),
             false);
  chunkPacket.chunk_index = chunkIndex;
  chunkPacket.chunk_len = len > 200 ? 200 : len;
  memcpy(chunkPacket.data, data, chunkPacket.chunk_len);
  return sendEspNowToAntenna(reinterpret_cast<const uint8_t *>(&chunkPacket), sizeof(chunkPacket));
}

bool recordAndStreamWavToAntenna(uint8_t triggerType, uint32_t *wavBytes) {
  if (!i2sReady) {
    lastRecordError = "I2S not ready";
    Serial.println("Cannot record: I2S mic is not ready.");
    return false;
  }

  lastRecordError = "";
  *wavBytes = AUDIO_TOTAL_BYTES;
  displayLCDStatus("Get ready", "Recording soon");
  ringFlash(0, 64, 0, 2, 120, 80);
  delay(SPEAK_DELAY_MS);

  if (!sendAudioDockTrigger(triggerType, AUDIO_TOTAL_BYTES)) {
    lastRecordError = "Trigger failed";
    return false;
  }
  delay(50);

  uint8_t wavHeader[WAV_HEADER_BYTES];
  writeWavHeader(wavHeader, AUDIO_DATA_BYTES);

  uint32_t chunkIndex = 0;
  if (!sendAudioDockChunk(wavHeader, WAV_HEADER_BYTES, chunkIndex++)) {
    lastRecordError = "Header failed";
    return false;
  }

  flushI2S();
  displayLCDStatus("Recording...", "Speak now!");
  Serial.printf("RECORD_STREAM_START seconds=%u bytes=%u\n",
                (unsigned int)RECORD_SECONDS,
                (unsigned int)AUDIO_TOTAL_BYTES);

  const uint32_t totalSamples = RECORD_SECONDS * SAMPLE_RATE;
  uint32_t samplesWritten = 0;
  uint32_t lastRingUpdate = 0;
  int32_t i2sBuffer[100];
  int16_t pcmBuffer[100];
  ringProgress(0, totalSamples, 80, 24, 0);

  while (samplesWritten < totalSamples) {
    uint32_t samplesToRead = totalSamples - samplesWritten;
    if (samplesToRead > 100) samplesToRead = 100;

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT,
                             i2sBuffer,
                             samplesToRead * sizeof(int32_t),
                             &bytesRead,
                             portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0) {
      lastRecordError = "I2S read failed";
      Serial.printf("I2S stream read failed: err=%d bytes=%u at_sample=%u\n",
                    err,
                    (unsigned int)bytesRead,
                    (unsigned int)samplesWritten);
      return false;
    }

    uint16_t samples = bytesRead / sizeof(int32_t);
    for (uint16_t i = 0; i < samples; i++) {
      int32_t sample16 = i2sBuffer[i] >> 16;
      int32_t boosted = sample16 * RECORD_GAIN;
      if (boosted > 32767) boosted = 32767;
      if (boosted < -32768) boosted = -32768;
      pcmBuffer[i] = (int16_t)boosted;
    }

    if (!sendAudioDockChunk(reinterpret_cast<const uint8_t *>(pcmBuffer),
                            samples * sizeof(int16_t),
                            chunkIndex++)) {
      lastRecordError = "Chunk failed";
      return false;
    }

    samplesWritten += samples;
    if (millis() - lastRingUpdate >= 120 || samplesWritten >= totalSamples) {
      ringProgress(samplesWritten, totalSamples, 80, 24, 0);
      lastRingUpdate = millis();
    }
  }

  Serial.printf("RECORD_STREAM_DONE wav_bytes=%u chunks=%u\n",
                (unsigned int)AUDIO_TOTAL_BYTES,
                (unsigned int)chunkIndex);
  return true;
}

bool streamWavFileToAntenna(uint8_t triggerType, uint32_t wavBytes) {
  if (!sdReady && !initSDCard()) {
    lastRecordError = "SD not ready";
    return false;
  }

  File file = SD.open(AUDIO_WAV_PATH, FILE_READ);
  if (!file) {
    lastRecordError = "SD read failed";
    Serial.println("Failed to open SD WAV file for reading.");
    return false;
  }

  if (wavBytes == 0) {
    wavBytes = file.size();
  }

  displayLCDStatus("Status: Sending", "Uploading...");
  if (!sendAudioDockTrigger(triggerType, wavBytes)) {
    lastRecordError = "Trigger failed";
    file.close();
    return false;
  }
  delay(50);

  Serial.printf("SD_STREAM_START bytes=%u path=%s\n",
                (unsigned int)wavBytes,
                AUDIO_WAV_PATH);

  uint8_t chunk[200];
  uint32_t sentBytes = 0;
  uint32_t chunkIndex = 0;
  uint32_t lastRingUpdate = 0;
  ringProgress(0, wavBytes, 0, 48, 64);

  while (sentBytes < wavBytes && file.available()) {
    uint32_t remaining = wavBytes - sentBytes;
    uint16_t want = remaining > sizeof(chunk) ? sizeof(chunk) : remaining;
    int bytesRead = file.read(chunk, want);
    if (bytesRead <= 0) {
      lastRecordError = "SD read failed";
      file.close();
      return false;
    }

    if (!sendAudioDockChunk(chunk, (uint16_t)bytesRead, chunkIndex++)) {
      lastRecordError = "Chunk failed";
      file.close();
      return false;
    }

    sentBytes += (uint32_t)bytesRead;
    if (millis() - lastRingUpdate >= 120 || sentBytes >= wavBytes) {
      ringProgress(sentBytes, wavBytes, 0, 48, 64);
      lastRingUpdate = millis();
    }

    delay(20);
  }

  file.close();
  Serial.printf("SD_STREAM_DONE sent=%u chunks=%u\n",
                (unsigned int)sentBytes,
                (unsigned int)chunkIndex);
  return sentBytes >= wavBytes;
}

void handleLocalSerialCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "LEDTEST" || command == "L") {
    runLedRingSelfTest();
  } else if (command == "SPEAKERTEST" || command == "SPKTEST" || command == "S") {
    runSpeakerSelfTest();
  } else if (command == "HELP" || command == "H") {
    Serial.println("Audio Dock commands: LEDTEST, SPEAKERTEST");
  } else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
    Serial.println("Use LEDTEST or SPEAKERTEST.");
  }
}

void pollLocalSerialCommands() {
  if (!Serial.available()) {
    return;
  }
  String command = Serial.readStringUntil('\n');
  handleLocalSerialCommand(command);
}

// -------------------- Main Arduino Setup & Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  ledRing.begin();
  ledRing.setBrightness(LED_RING_BRIGHTNESS);
  ringClear();
  ringShowReady();
  disableOnboardLed();

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

  if (!initSDCard()) {
    displayLCDStatus("SD Card Error", "Check wiring");
    ringShowError();
    delay(2000);
  }

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
  addEspNowPeer(ESPNOW_BROADCAST_MAC);

  displayLCDStatus("System Ready", "Clap to speak!");
  currentState = STATE_WAIT_CLAP;
}

void loop() {
  pollLocalSerialCommands();

  switch (currentState) {
    case STATE_WAIT_CLAP: {
      uint8_t mac[6] = {};
      WiFi.macAddress(mac);
      char macLcd[18];
      snprintf(macLcd, sizeof(macLcd), "MAC:%02X%02X%02X%02X%02X%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      displayLCDStatus("Clap to speak!", macLcd);
      ringChase(0, 0, 80, 0);
      
      if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
        displayLCDStatus("Inference Err", "Memory failed");
        ringShowError();
        delay(3000);
        break;
      }

      bool clapDetected = false;
      uint8_t triggerType = 0; // 1 = Single, 2 = Double
      uint16_t listeningStep = 0;

      while (!clapDetected) {
        pollLocalSerialCommands();
        ringChase(0, 0, 80, listeningStep++);

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

        float singleClapScore = 0.0f;
        float doubleClapScore = 0.0f;
        float noiseScore = 0.0f;
        float bestScore = -1.0f;
        String bestLabel = "";

        if (PRINT_CLAP_SCORES) {
          Serial.print("CLAP_SCORES");
        }

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          String label = String(result.classification[ix].label);
          float val = result.classification[ix].value;

          if (PRINT_CLAP_SCORES) {
            Serial.printf(" %s=%.2f", label.c_str(), val);
          }

          if (val > bestScore) {
            bestScore = val;
            bestLabel = label;
          }

          if (label.equalsIgnoreCase("Single clap")) {
            singleClapScore = val;
          } else if (label.equalsIgnoreCase("Double clap")) {
            doubleClapScore = val;
          } else if (label.equalsIgnoreCase("Noice") || label.equalsIgnoreCase("Noise")) {
            noiseScore = val;
          }
        }

        float combinedClapScore = singleClapScore + doubleClapScore;
        bool bestIsClap = bestLabel.equalsIgnoreCase("Single clap") || bestLabel.equalsIgnoreCase("Double clap");
        bool strongSingleOrDouble = singleClapScore >= CLAP_LABEL_THRESHOLD || doubleClapScore >= CLAP_LABEL_THRESHOLD;
        bool combinedClapWins = combinedClapScore >= CLAP_COMBINED_THRESHOLD && combinedClapScore > noiseScore;

        if (PRINT_CLAP_SCORES) {
          uint32_t peak = lastInferencePeak;
          Serial.printf(" combined=%.2f best=%s %.2f peak=%lu\n",
                        combinedClapScore,
                        bestLabel.c_str(),
                        bestScore,
                        (unsigned long)peak);
        }

        if ((bestIsClap && strongSingleOrDouble) || combinedClapWins) {
          triggerType = doubleClapScore > singleClapScore ? 2 : 1;
          const char *triggerLabel = triggerType == 2 ? "Double clap" : "Single clap";
          float triggerScore = triggerType == 2 ? doubleClapScore : singleClapScore;
          Serial.printf("Triggered! Detected: %s score=%.2f combined=%.2f noise=%.2f\n",
                        triggerLabel,
                        triggerScore,
                        combinedClapScore,
                        noiseScore);
          clapDetected = true;
        }
      }

      // Clap detected! Stop model classification
      microphone_inference_end();
      displayLCDStatus("Clap Detected!", "Speak now...");
      ringFlash(0, 64, 0, 2, 120, 80);
      delay(250);

      pendingTriggerType = triggerType;
      currentState = STATE_RECORDING;
      break;
    }

    case STATE_RECORDING: {
      displayLCDStatus("Record+Send", "Listening...");
      
      if (!initI2SMic()) {
        displayLCDStatus("Mic Error", "Failed to start");
        ringShowError();
        delay(2000);
        currentState = STATE_WAIT_CLAP;
        break;
      }

      uint32_t wavSize = 0;
      bool recordedOk = recordWavToSD(&wavSize);
      
      // Uninstall microphone I2S driver
      i2s_driver_uninstall(I2S_PORT);
      i2sReady = false;

      if (!recordedOk) {
        String errorLine = lastRecordError.length() > 0 ? lastRecordError : "Failed saving";
        displayLCDStatus("Record Error", errorLine.substring(0, 16));
        ringShowError();
        delay(2000);
        currentState = STATE_WAIT_CLAP;
        break;
      }

      recordedWavBytes = wavSize;
      currentState = STATE_STREAMING;
      break;
    }

    case STATE_STREAMING: {
      bool streamedOk = streamWavFileToAntenna(pendingTriggerType, recordedWavBytes);
      if (!streamedOk) {
        String errorLine = lastRecordError.length() > 0 ? lastRecordError : "Send failed";
        displayLCDStatus("Send Error", errorLine.substring(0, 16));
        ringShowError();
        delay(2000);
        currentState = STATE_WAIT_CLAP;
        break;
      }

      transcriptReceived = false;
      transcriptWaitStartMs = millis();
      currentState = STATE_WAIT_TRANSCRIPT;
      break;
    }

    case STATE_WAIT_TRANSCRIPT: {
      displayLCDStatus("Status: Wait...", "Transcribing...");
      uint16_t waitStep = 0;
      uint32_t lastRingUpdate = 0;
      
      while (!transcriptReceived && (millis() - transcriptWaitStartMs < TRANSCRIPT_TIMEOUT_MS)) {
        pollLocalSerialCommands();
        if (millis() - lastRingUpdate >= 100) {
          ringChase(64, 0, 80, waitStep++);
          lastRingUpdate = millis();
        }
        delay(50);
      }

      if (transcriptReceived) {
        displayLCDTranscript(lastTranscriptText);
        ringShowSuccess();
        playTranscriptionDoneSound();
        delay(5000); // Display the transcript for 5 seconds
      } else {
        displayLCDStatus("Status: Timeout", "No response");
        ringShowError();
        delay(3000);
      }

      ringShowReady();
      currentState = STATE_WAIT_CLAP;
      break;
    }
  }
}
