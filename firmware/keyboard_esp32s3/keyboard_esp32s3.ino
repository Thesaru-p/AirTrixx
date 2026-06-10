#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#if __has_include("../shared/AirTrixxConfig.h")
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
#else
#include "AirTrixxConfig.h"
#include "AirTrixxProtocol.h"
#endif

// ============================================================
// WORDS LIST - direct firmware fuzzy matching fallback
// ============================================================
const char* wordList[] = {
  "the", "and", "you", "that", "was", "for", "are", "with",
  "his", "they", "this", "have", "from", "one", "had", "but",
  "not", "what", "all", "were", "when", "your", "can", "said",
  "there", "use", "each", "which", "she", "how", "their", "will",
  "other", "about", "out", "many", "then", "them", "these", "some",
  "her", "would", "make", "like", "him", "into", "time", "has",
  "look", "two", "more", "write", "see", "number", "way", "could",
  "come", "its", "over", "think", "also", "back", "after", "work",
  "first", "well", "even", "want", "because", "any", "give", "most",
  "tell", "very", "just", "name", "good", "great", "where", "help",
  "through", "much", "before", "right", "mean", "old", "same",
  "boy", "follow", "came", "show", "around", "form", "three",
  "small", "set", "put", "end", "does", "another", "large",
  "need", "big", "high", "such", "place", "turn", "here", "why",
  "ask", "went", "read", "land", "different", "home", "move",
  "try", "kind", "hand", "again", "change", "off", "play",
  "air", "away", "animal", "house", "point", "page", "letter",
  "found", "study", "still", "learn", "plant", "food", "sun",
  "four", "thought", "let", "keep", "feet", "side",
  "paper", "together", "got", "group", "often", "run", "important",
  "car", "mile", "night", "walk", "white", "sea", "began", "grow",
  "river", "carry", "state", "once", "book", "hear", "stop",
  "second", "late", "idea", "body", "dog", "cat", "eat", "face",
  "door", "cut", "sure", "watch", "color", "note", "rain", "road",
  "farm", "pull", "draw", "voice", "seen", "cold", "plan",
  "sing", "war", "ground", "fall", "king", "town", "fire",
  "problem", "piece", "told", "knew", "pass", "since", "top",
  "whole", "space", "heard", "best", "hour", "better", "true",
  "hundred", "five", "remember", "step", "early", "hold", "west",
  "reach", "fast", "listen", "feel", "talk", "bird", "soon",
  "family", "leave", "song", "measure", "product", "black", "short",
  "wind", "rock", "lived", "happened", "horse", "stood", "strong",
  "hello", "world", "please", "thank", "sorry", "okay", "yes", "no",
  "maybe", "today", "tomorrow", "morning", "love", "hate",
  "happy", "sad", "nice", "cool", "hot",
  "phone", "send", "call", "meet", "free", "busy", "wait", "done",
  "open", "close", "start", "buy", "sell", "pay", "get",
  "take", "bring", "find", "know",
  "going", "coming", "doing", "having", "being", "making",
  "money", "people", "year", "day", "week", "month",
  "water", "light", "dark", "long", "slow", "hard", "easy", "new", "young"
};
const int WORD_COUNT = sizeof(wordList) / sizeof(wordList[0]);

// ============================================================
// CONFIG
// ============================================================
#define NUM_SENSORS            4
#define TCA_ADDR               0x70
#define AVG_WINDOW             7
#define MIN_AVG_READY          4
#define CALIBRATION_SAMPLES    25
#define IDLE_CALIBRATION_WAIT_MS 3000
#define CAPS_DISTANCE_MM       50
#define KEY_DWELL_MS           80
#define CAPS_HOLD_MS           300
#define NUM_HOLD_MS            300
#define NUM_MODE_COOLDOWN_MS   1200
#define COVER_TRIGGER_MM       15
#define DEFAULT_MAX_SIGNAL_DISTANCE_MM 210
#define COVER_CALIBRATION_WAIT_MS 3000
#define COVER_LIMIT_MARGIN_MM 5
#define MAX_KEY_MARGIN_MM      10
#define MEASUREMENT_BUDGET_US  20000
#define TCA_SETTLE_US          700
#define SENSOR_INIT_RETRIES    3
#define LOG_SAMPLE_INTERVAL_MS 20
#define LOG_LABEL_MAX_LEN      32
#define DOMINANT_MARGIN_MM     20

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)
  #define I2C_SDA_PIN 8
  #define I2C_SCL_PIN 9
#elif defined(ESP32)
  #define I2C_SDA_PIN 21
  #define I2C_SCL_PIN 22
#else
  #define I2C_SDA_PIN 8
  #define I2C_SCL_PIN 9
#endif

// ============================================================
// GLOBALS
// ============================================================
#define NUM_KEY_MAPS 4
Adafruit_VL53L0X lox[NUM_SENSORS];
bool loxReady[NUM_SENSORS] = {false, false, false, false};

int   zeroMM[NUM_SENSORS]     = {0, 0, 0, 0};
int   idleMM[NUM_SENSORS]     = {9999, 9999, 9999, 9999};
bool  idleValid[NUM_SENSORS]  = {false, false, false, false};
int   maxSignalMM[NUM_SENSORS] = {
  DEFAULT_MAX_SIGNAL_DISTANCE_MM,
  DEFAULT_MAX_SIGNAL_DISTANCE_MM,
  DEFAULT_MAX_SIGNAL_DISTANCE_MM,
  DEFAULT_MAX_SIGNAL_DISTANCE_MM
};

int     avgBuffer[NUM_SENSORS][AVG_WINDOW] = {};
long    avgSum[NUM_SENSORS]   = {0, 0, 0, 0};
uint8_t avgPos[NUM_SENSORS]   = {0, 0, 0, 0};
uint8_t avgCount[NUM_SENSORS] = {0, 0, 0, 0};

bool capsMode           = false;
bool capsGestureLatched = false;
unsigned long capsStartMs = 0;

bool numbersMode          = false;
bool numGestureLatched    = false;
unsigned long numStartMs  = 0;
unsigned long lastNumToggleMs = 0;

bool  loggingMode         = false;
char  logLabel[LOG_LABEL_MAX_LEN] = "";
bool  logHeaderPrinted    = false;
unsigned long lastLogMs   = 0;
unsigned long logFrame    = 0;

char  pendingKey          = '\0';
char  lastPrintedKey      = '\0';
unsigned long pendingStartMs  = 0;
unsigned long lastKeypressMs  = 0;
String swipeBuffer        = "";
int prevCoveredCount      = 0;
uint16_t airTrixxKeyboardSequence = 0;
uint16_t airTrixxKeyboardBatterySequence = 0;
unsigned long lastAirTrixxReportMs = 0;
unsigned long lastAirTrixxBatteryReportMs = 0;
bool airTrixxBatteryReportSent = false;
volatile bool airTrixxRecalibrationRequested = false;

// Exact horizontal key centers measured from sensors on the right side.
// Distance starts at P/L/Backspace/Return and increases toward the left side.
const uint8_t TOP_CENTERS[]       = {10, 30, 50, 70, 90, 110, 130, 150, 170, 190};
const uint8_t TOP_HALF_WIDTHS[]   = { 9,  9,  9,  9,  9,   9,   9,   9,   9,   9};
const uint8_t HOME_CENTERS[]      = {20, 40, 60, 80, 100, 120, 140, 160, 180};
const uint8_t HOME_HALF_WIDTHS[]  = { 9,  9,  9,  9,   9,   9,   9,   9,   9};
const uint8_t LOWER_CENTERS[]     = {13, 40, 60, 80, 100, 120, 140, 160, 188};
const uint8_t LOWER_HALF_WIDTHS[] = {12,  9,  9,  9,   9,   9,   9,   9,  12};
const uint8_t CTRL_CENTERS[]      = {25, 100, 175};
const uint8_t CTRL_HALF_WIDTHS[]  = {24,  49,  24};

// Special actions: ^ = Shift, < = Backspace, # = ?123, space = Space, \r = Return.
struct ChannelMap {
  uint8_t ch;
  const char *actions;
  const char *labels;
  const uint8_t *centers;
  const uint8_t *halfWidths;
  uint8_t count;
};
ChannelMap keyMaps[] = {
  {1, "poiuytrewq", "P O I U Y T R E W Q", TOP_CENTERS, TOP_HALF_WIDTHS, 10},
  {2, "lkjhgfdsa",  "L K J H G F D S A", HOME_CENTERS, HOME_HALF_WIDTHS, 9},
  {0, "<mnbvcxz^",  "BACKSPACE M N B V C X Z SHIFT", LOWER_CENTERS, LOWER_HALF_WIDTHS, 9},
  {3, "\r #",       "RETURN SPACE ?123", CTRL_CENTERS, CTRL_HALF_WIDTHS, 3}
};

void printChannelMappings();
void printDetectionLimits();
void sendAirTrixxKeyboardBattery(bool force = false);

// ============================================================
// FUZZY MATCHING
// ============================================================
int editDistance(const char* a, const char* b) {
  int la = min((int)strlen(a), 32);
  int lb = min((int)strlen(b), 32);
  int dp[33][33];
  for (int i = 0; i <= la; i++) dp[i][0] = i;
  for (int j = 0; j <= lb; j++) dp[0][j] = j;
  for (int i = 1; i <= la; i++)
    for (int j = 1; j <= lb; j++)
      dp[i][j] = (tolower((unsigned char)a[i-1]) == tolower((unsigned char)b[j-1]))
        ? dp[i-1][j-1]
        : 1 + min(dp[i-1][j-1], min(dp[i-1][j], dp[i][j-1]));
  return dp[la][lb];
}

void extractConsonants(const char* word, char* out, int maxLen) {
  const char vowels[] = "aeiouAEIOU";
  int j = 0;
  for (int i = 0; word[i] && j < maxLen-1; i++)
    if (!strchr(vowels, word[i])) out[j++] = tolower((unsigned char)word[i]);
  out[j] = '\0';
}

int matchScore(const char* swipe, const char* candidate) {
  int direct = editDistance(swipe, candidate);
  char ss[32], cs[32];
  extractConsonants(swipe, ss, sizeof(ss));
  extractConsonants(candidate, cs, sizeof(cs));
  int skel    = editDistance(ss, cs);
  int lenDiff = abs((int)strlen(swipe) - (int)strlen(candidate));
  return min(direct, skel) + (lenDiff > 3 ? lenDiff : 0);
}

String fuzzyMatchWord(String swipe) {
  if (swipe.length() == 0) return swipe;
  char swipeLower[32];
  swipe.toLowerCase();
  swipe.toCharArray(swipeLower, sizeof(swipeLower));

  int bestScore = 9999; const char* bestWord = nullptr;
  int secScore  = 9999; const char* secWord  = nullptr;

  for (int i = 0; i < WORD_COUNT; i++) {
    int s = matchScore(swipeLower, wordList[i]);
    if (s < bestScore) { secScore = bestScore; secWord = bestWord; bestScore = s; bestWord = wordList[i]; }
    else if (s < secScore) { secScore = s; secWord = wordList[i]; }
  }

  Serial.print("  [Fuzzy] Best: ");
  if (bestWord) { Serial.print(bestWord); Serial.print("("); Serial.print(bestScore); Serial.print(")"); }
  if (secWord)  { Serial.print(" 2nd: "); Serial.print(secWord); Serial.print("("); Serial.print(secScore); Serial.print(")"); }
  Serial.println();

  return (bestScore <= 4 && bestWord) ? String(bestWord) : swipe;
}

// ============================================================
// SENSOR HELPERS
// ============================================================
uint8_t chToIdx(uint8_t ch) {
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) if (keyMaps[i].ch == ch) return i;
  return 255;
}

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(TCA_SETTLE_US);
}

bool readRaw(uint8_t ch, int &distanceMM) {
  VL53L0X_RangingMeasurementData_t measure;
  tcaSelect(ch);
  uint8_t idx = chToIdx(ch);
  if (idx == 255) return false;
  if (!loxReady[idx]) return false;
  lox[idx].rangingTest(&measure, false);
  if (measure.RangeStatus == 4 || measure.RangeMilliMeter <= 0 || measure.RangeMilliMeter >= 8190) return false;
  distanceMM = measure.RangeMilliMeter;
  return true;
}

void sortValues(int v[], int n) {
  for (int i = 1; i < n; i++) {
    int key = v[i], j = i-1;
    while (j >= 0 && v[j] > key) { v[j+1] = v[j]; j--; }
    v[j+1] = key;
  }
}

bool readTrimmedAverage(uint8_t ch, int &avgMM, int samples, int &valid) {
  int values[CALIBRATION_SAMPLES];
  valid = 0;
  for (int i = 0; i < samples; i++) { int d; if (readRaw(ch, d)) values[valid++] = d; delay(2); }
  if (valid < (samples/2 + 1)) return false;
  sortValues(values, valid);
  int start = valid >= 5 ? 1 : 0, end = valid >= 5 ? valid-1 : valid;
  long total = 0;
  for (int i = start; i < end; i++) total += values[i];
  avgMM = total / (end - start);
  return true;
}

void resetMovingAverages() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { avgSum[i] = 0; avgPos[i] = 0; avgCount[i] = 0; }
}

void addMovingSample(uint8_t ch, int mm) {
  uint8_t i = chToIdx(ch); if (i == 255) return;
  if (avgCount[i] < AVG_WINDOW) { avgBuffer[i][avgPos[i]] = mm; avgSum[i] += mm; avgCount[i]++; }
  else { avgSum[i] -= avgBuffer[i][avgPos[i]]; avgBuffer[i][avgPos[i]] = mm; avgSum[i] += mm; }
  avgPos[i] = (avgPos[i] + 1) % AVG_WINDOW;
}

bool getMovingAverage(uint8_t ch, int &avgMM) {
  uint8_t i = chToIdx(ch); if (i == 255 || avgCount[i] < MIN_AVG_READY) return false;
  avgMM = avgSum[i] / avgCount[i]; return true;
}

// ============================================================
// KEY HELPERS
// ============================================================
int getKeyCount(uint8_t ch) {
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) if (keyMaps[i].ch == ch) return keyMaps[i].count;
  return 0;
}

char applyCaps(char key) { return (capsMode && key >= 'a' && key <= 'z') ? toupper(key) : key; }

int distanceToKeyIndex(const ChannelMap &map, int d) {
  for (uint8_t i = 0; i < map.count; i++) {
    int rangeStart = (i == 0) ? 0 : (map.centers[i - 1] + map.centers[i] + 1) / 2;
    int rangeEnd = (i == map.count - 1)
      ? map.centers[i] + map.halfWidths[i]
      : (map.centers[i] + map.centers[i + 1] + 1) / 2 - 1;
    if (d >= rangeStart && d <= rangeEnd) return i;
  }
  return -1;
}

char getActionForChannel(uint8_t ch, int d) {
  if (d < 0) d = 0;
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    if (keyMaps[i].ch == ch) {
      int idx = distanceToKeyIndex(keyMaps[i], d);
      if (idx < 0) return '\0';
      return applyCaps(keyMaps[i].actions[idx]);
    }
  }
  return '\0';
}

int getMaxKeyDistance(uint8_t ch) {
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    if (keyMaps[i].ch == ch) {
      uint8_t last = keyMaps[i].count - 1;
      return keyMaps[i].centers[last] + keyMaps[i].halfWidths[last];
    }
  }
  return -1;
}

bool isRealCover(uint8_t ch, int rawMM) {
  uint8_t idx = chToIdx(ch);
  if (idx == 255) return false;
  int maxKeyRawMM = zeroMM[idx] + getMaxKeyDistance(ch) + MAX_KEY_MARGIN_MM;
  if (rawMM - zeroMM[idx] > maxSignalMM[idx]) return false;
  if (rawMM > maxKeyRawMM) return false;
  if (idleValid[idx]) return rawMM < (idleMM[idx] - COVER_TRIGGER_MM);
  return true;
}

void finishSwipeBuffer(const char *actionLabel) {
  if (swipeBuffer.length() > 0) {
    Serial.print("RAW SWIPE: "); Serial.println(swipeBuffer);
    String matched = fuzzyMatchWord(swipeBuffer);
    Serial.print("WORD: "); Serial.println(matched);
    swipeBuffer = "";
  }
  if (actionLabel) Serial.println(actionLabel);
}

void handleKeyAction(char action, uint8_t ch, int distanceMM) {
  if (action == '^') {
    capsMode = !capsMode;
    Serial.println(capsMode ? "SHIFT/CAPS ON" : "SHIFT/CAPS OFF");
    return;
  }
  if (action == '<') {
    if (swipeBuffer.length() > 0) swipeBuffer.remove(swipeBuffer.length() - 1);
    Serial.print("BACKSPACE -> "); Serial.println(swipeBuffer);
    return;
  }
  if (action == ' ') {
    finishSwipeBuffer("SPACE");
    return;
  }
  if (action == '\r') {
    finishSwipeBuffer("RETURN");
    return;
  }
  if (action == '#') {
    numbersMode = !numbersMode;
    Serial.println(numbersMode ? "?123 MODE REQUESTED" : "ABC MODE REQUESTED");
    return;
  }

  if (swipeBuffer.length() == 0 || swipeBuffer.charAt(swipeBuffer.length() - 1) != action)
    swipeBuffer += action;
  Serial.print("Buffered: "); Serial.print(swipeBuffer);
  Serial.print("  CH"); Serial.print(ch);
  Serial.print("="); Serial.print(distanceMM); Serial.println("mm");
}

void resetKeyState() { pendingKey = '\0'; lastPrintedKey = '\0'; pendingStartMs = 0; }

// ============================================================
// DOMINANT SENSOR SELECTION
// ============================================================
int findDominantSensor(int distPerCh[], bool rawValid[]) {
  int winnerIdx = -1;
  int winnerDistance = 9999;

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    if (distPerCh[i] < 0) continue;
    if (!rawValid[i])      continue;
    if (distPerCh[i] < winnerDistance) {
      winnerDistance = distPerCh[i];
      winnerIdx  = i;
    }
  }

  if (winnerIdx < 0) return -1;

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    if ((int)i == winnerIdx) continue;
    if (distPerCh[i] < 0)   continue;
    if (!rawValid[i])        continue;
    int gap = distPerCh[i] - winnerDistance;
    if (gap < DOMINANT_MARGIN_MM) return -1;
  }

  return winnerIdx;
}

// ============================================================
// LOGGING
// ============================================================
void printLogHeader() {
  Serial.print("frame,ms,label");
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) { Serial.print(",ch"); Serial.print(keyMaps[i].ch); Serial.print("_raw"); }
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) { Serial.print(",ch"); Serial.print(keyMaps[i].ch); Serial.print("_mm"); }
  Serial.println();
}

void printLogRow(int rawMM[], bool rawValid[], int relativeMM[]) {
  Serial.print(logFrame++); Serial.print(','); Serial.print(millis()); Serial.print(','); Serial.print(logLabel);
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) { Serial.print(','); if (rawValid[i]) Serial.print(rawMM[i]); else Serial.print(-1); }
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) { Serial.print(','); Serial.print(relativeMM[i]); }
  Serial.println();
}

// ============================================================
// SERIAL COMMANDS
// ============================================================
bool eqIC(const char *a, const char *b) {
  while (*a && *b) { if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false; a++; b++; }
  return !*a && !*b;
}
bool swIC(const char *t, const char *p) {
  while (*p) { if (!*t || tolower((unsigned char)*t) != tolower((unsigned char)*p)) return false; t++; p++; }
  return true;
}

void handleSerialCommand() {
  static char line[64]; static uint8_t pos = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      line[pos] = '\0'; pos = 0;
      if (!line[0]) continue;
      if      (eqIC(line, "help"))    Serial.println("Commands: ai on | ai off | log on | log off | label <text> | clear | map");
      else if (eqIC(line, "map"))     printChannelMappings();
      else if (eqIC(line, "ai on"))   { loggingMode = true; logHeaderPrinted = false; lastLogMs = 0; Serial.println("AI RAW STREAM ON"); }
      else if (eqIC(line, "ai off"))  { loggingMode = false; Serial.println("AI RAW STREAM OFF"); }
      else if (eqIC(line, "log on"))  { loggingMode = true; logHeaderPrinted = false; lastLogMs = 0; Serial.println("LOGGING ON"); }
      else if (eqIC(line, "log off")) { loggingMode = false; Serial.println("LOGGING OFF"); }
      else if (eqIC(line, "clear"))   { logLabel[0] = '\0'; Serial.println("LABEL CLEARED"); }
      else if (swIC(line, "label "))  { strncpy(logLabel, line+6, LOG_LABEL_MAX_LEN-1); logLabel[LOG_LABEL_MAX_LEN-1] = '\0'; Serial.print("LABEL SET: "); Serial.println(logLabel); }
      else if (eqIC(line, "dist") || eqIC(line, "measure")) {
        // Print a single-shot distance reading for each channel (mm), -1 = invalid
        for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
          uint8_t ch = keyMaps[i].ch;
          int d = -1;
          tcaSelect(ch);
          if (readRaw(ch, d)) {
            Serial.print("CH"); Serial.print(ch); Serial.print("="); Serial.print(d);
          } else {
            Serial.print("CH"); Serial.print(ch); Serial.print("=-1");
          }
          if (i < NUM_KEY_MAPS - 1) Serial.print(", ");
        }
        Serial.println();
      }
      else if (eqIC(line, "limits")) {
        printDetectionLimits();
      }
      else                            Serial.println("Unknown command. Type: help");
    } else if (pos < sizeof(line)-1) line[pos++] = c;
  }
}

// ============================================================
// CALIBRATION
// ============================================================
void haltWithError(const char *msg) {
  Serial.print("FATAL ERROR: "); Serial.println(msg);
  while (1) {
    Serial.print("FATAL ERROR: "); Serial.println(msg);
    Serial.println("System halted. Reset to retry.");
    delay(3000);
  }
}

void printChannelMappings() {
  Serial.println("Channel key mappings:");
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    Serial.print("  CH");
    Serial.print(keyMaps[i].ch);
    Serial.print(" -> ");
    Serial.println(keyMaps[i].labels);
    Serial.print("    accepted ranges: ");
    for (uint8_t j = 0; j < keyMaps[i].count; j++) {
      if (j > 0) Serial.print(" | ");
      int rangeStart = (j == 0) ? 0 : (keyMaps[i].centers[j - 1] + keyMaps[i].centers[j] + 1) / 2;
      int rangeEnd = (j == keyMaps[i].count - 1)
        ? keyMaps[i].centers[j] + keyMaps[i].halfWidths[j]
        : (keyMaps[i].centers[j] + keyMaps[i].centers[j + 1] + 1) / 2 - 1;
      Serial.print(rangeStart);
      Serial.print('-');
      Serial.print(rangeEnd);
      Serial.print("mm");
    }
    Serial.println();
  }
}

void printDetectionLimits() {
  Serial.print("DETECT_LIMITS_MM");
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    Serial.print(",ch");
    Serial.print(keyMaps[i].ch);
    Serial.print("=");
    Serial.print(maxSignalMM[i]);
  }
  Serial.println();
}

void calibrateDetectionLimits() {
  Serial.println("COVER DISTANCE CALIBRATION");
  Serial.println("Cover all sensors at the farthest distance you want detected.");
  Serial.print("Starting in ");
  Serial.print(COVER_CALIBRATION_WAIT_MS / 1000);
  Serial.println(" seconds...");
  delay(COVER_CALIBRATION_WAIT_MS);

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    uint8_t ch = keyMaps[i].ch;
    if (!loxReady[i]) {
      Serial.print("CH"); Serial.print(ch);
      Serial.println(" not ready, using default detect limit");
      continue;
    }
    int d;
    int validReadings = 0;
    if (readTrimmedAverage(ch, d, CALIBRATION_SAMPLES, validReadings)) {
      maxSignalMM[i] = d + COVER_LIMIT_MARGIN_MM;
      Serial.print("CH"); Serial.print(ch);
      Serial.print(" covered distance = "); Serial.print(d);
      Serial.print("mm, detect limit = "); Serial.print(maxSignalMM[i]);
      Serial.println("mm");
    } else {
      Serial.print("CH"); Serial.print(ch);
      Serial.print(" cover calibration failed with ");
      Serial.print(validReadings);
      Serial.print("/");
      Serial.print(CALIBRATION_SAMPLES);
      Serial.println(" valid readings, using default detect limit");
    }
  }
  printDetectionLimits();
}

void calibrateIdleBackground() {
  Serial.println("AUTOMATIC IDLE CALIBRATION");
  Serial.println("Remove hands and objects from the keyboard.");
  Serial.print("Starting in ");
  Serial.print(IDLE_CALIBRATION_WAIT_MS / 1000);
  Serial.println(" seconds...");
  delay(IDLE_CALIBRATION_WAIT_MS);

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    uint8_t ch = keyMaps[i].ch;
    if (!loxReady[i]) {
      Serial.print("CH"); Serial.print(ch);
      Serial.println(" not ready, skipping idle calibration");
      continue;
    }
    int d;
    int validReadings = 0;
    if (readTrimmedAverage(ch, d, CALIBRATION_SAMPLES, validReadings)) {
      idleMM[i] = d;
      zeroMM[i] = 0;
      idleValid[i] = true;
      Serial.print("CH"); Serial.print(ch);
      Serial.print(" idle/background = "); Serial.print(d);
      Serial.println("mm");
    } else {
      idleValid[i] = false;
      Serial.print("CH"); Serial.print(ch);
      Serial.print(" idle calibration failed with ");
      Serial.print(validReadings);
      Serial.print("/");
      Serial.print(CALIBRATION_SAMPLES);
      Serial.println(" valid readings");
    }
  }
  resetMovingAverages();
}

void calibrateSensors() {
  calibrateDetectionLimits();
  calibrateIdleBackground();
  Serial.println("Calibration complete");
}

uint8_t keyboardBatteryPercent(float voltage) {
  if (voltage <= KEYBOARD_BATTERY_EMPTY_V) {
    return 0;
  }
  if (voltage >= KEYBOARD_BATTERY_FULL_V) {
    return 100;
  }
  return static_cast<uint8_t>(lroundf(
    (voltage - KEYBOARD_BATTERY_EMPTY_V) * 100.0f /
    (KEYBOARD_BATTERY_FULL_V - KEYBOARD_BATTERY_EMPTY_V)
  ));
}

void setupKeyboardBatterySense() {
  pinMode(KEYBOARD_BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(KEYBOARD_BATTERY_ADC_PIN, ADC_11db);
#endif
  Serial.print("[KEYBOARD] Battery divider ADC GPIO=");
  Serial.print(KEYBOARD_BATTERY_ADC_PIN);
  Serial.print(", ratio=");
  Serial.println(KEYBOARD_BATTERY_DIVIDER_RATIO, 2);
}

bool readKeyboardBattery(float &voltage, uint16_t &adcRaw, uint16_t &senseMv) {
  const uint8_t samples = 16;
  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    rawSum += analogRead(KEYBOARD_BATTERY_ADC_PIN);
    mvSum += analogReadMilliVolts(KEYBOARD_BATTERY_ADC_PIN);
    delay(2);
  }
  adcRaw = static_cast<uint16_t>((rawSum + samples / 2) / samples);
  senseMv = static_cast<uint16_t>((mvSum + samples / 2) / samples);
  voltage = (senseMv / 1000.0f) * KEYBOARD_BATTERY_DIVIDER_RATIO;
  return senseMv > 100 && voltage >= 2.0f && voltage <= 4.6f;
}

void printMacAddress(const uint8_t mac[6]) {
  for (int i = 0; i < 6; ++i) {
    if (i > 0) Serial.print(":");
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
}

bool addAirTrixxPeer(const uint8_t mac[6]) {
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
    Serial.print("[KEYBOARD] ESP-NOW add antenna peer failed: ");
    Serial.println(result);
    return false;
  }
  return true;
}

void handleAirTrixxCommandPacket(const uint8_t *data, int len) {
  if (data == nullptr || len < static_cast<int>(sizeof(AirTrixxPacketHeader))) {
    return;
  }
  AirTrixxPacketHeader header = {};
  memcpy(&header, data, sizeof(header));
  if (header.protocol_version != AIRTRIXX_PROTOCOL_VERSION ||
      header.msg_type != MSG_KEYBOARD_COMMAND ||
      len != static_cast<int>(sizeof(KeyboardCommandPacket))) {
    return;
  }

  KeyboardCommandPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (packet.header.device_id != DEVICE_ANTENNA) {
    return;
  }
  if (packet.command == KEYBOARD_CMD_RECALIBRATE) {
    airTrixxRecalibrationRequested = true;
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onAirTrixxDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info;
  handleAirTrixxCommandPacket(incomingData, len);
}
#else
void onAirTrixxDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  (void)mac;
  handleAirTrixxCommandPacket(incomingData, len);
}
#endif

void initAirTrixxWireless() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  uint8_t mac[6] = {};
  WiFi.macAddress(mac);
  Serial.print("[KEYBOARD] WiFi STA MAC=");
  printMacAddress(mac);
  Serial.print(", channel=");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[KEYBOARD] ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onAirTrixxDataRecv);
  addAirTrixxPeer(ANTENNA_MAC_PLACEHOLDER);
}

void sendAirTrixxKeyboardTof(int rawMM[], bool rawValid[]) {
  unsigned long nowMs = millis();
  const unsigned long intervalMs = max(1UL, 1000UL / KEYBOARD_REPORT_HZ);
  if (nowMs - lastAirTrixxReportMs < intervalMs) {
    return;
  }
  lastAirTrixxReportMs = nowMs;

  KeyboardTofPacket packet = {};
  fillHeader(
    packet.header,
    MSG_KEYBOARD_TOF,
    DEVICE_KEYBOARD,
    ++airTrixxKeyboardSequence,
    nowMs,
    false
  );
  packet.distance_mm_1 = rawValid[0] && rawMM[0] > 0 ? rawMM[0] : 0;
  packet.distance_mm_2 = rawValid[1] && rawMM[1] > 0 ? rawMM[1] : 0;
  packet.distance_mm_3 = rawValid[2] && rawMM[2] > 0 ? rawMM[2] : 0;
  packet.distance_mm_4 = rawValid[3] && rawMM[3] > 0 ? rawMM[3] : 0;
  packet.valid_1 = rawValid[0] ? 1 : 0;
  packet.valid_2 = rawValid[1] ? 1 : 0;
  packet.valid_3 = rawValid[2] ? 1 : 0;
  packet.valid_4 = rawValid[3] ? 1 : 0;
  esp_now_send(ANTENNA_MAC_PLACEHOLDER, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

void sendAirTrixxKeyboardBattery(bool force) {
  unsigned long nowMs = millis();
  if (!force && airTrixxBatteryReportSent &&
      nowMs - lastAirTrixxBatteryReportMs < KEYBOARD_BATTERY_REPORT_MS) {
    return;
  }
  lastAirTrixxBatteryReportMs = nowMs;
  airTrixxBatteryReportSent = true;

  float batteryVoltage = 0.0f;
  uint16_t adcRaw = 0;
  uint16_t senseMv = 0;
  bool batteryValid = readKeyboardBattery(batteryVoltage, adcRaw, senseMv);
  uint8_t batteryPercent = batteryValid ? keyboardBatteryPercent(batteryVoltage) : 0;

  BatteryStatusPacket packet = {};
  fillHeader(
    packet.header,
    MSG_BATTERY_STATUS,
    DEVICE_KEYBOARD,
    ++airTrixxKeyboardBatterySequence,
    nowMs,
    batteryValid && batteryPercent <= 15
  );
  packet.battery_mv = batteryValid ? static_cast<uint16_t>(lroundf(batteryVoltage * 1000.0f)) : 0;
  packet.battery_percent = batteryPercent;
  packet.battery_valid = batteryValid ? 1 : 0;
  packet.battery_adc_raw = adcRaw;
  esp_now_send(ANTENNA_MAC_PLACEHOLDER, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.print("I2C SDA="); Serial.print(I2C_SDA_PIN); Serial.print(" SCL="); Serial.println(I2C_SCL_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  setupKeyboardBatterySense();
  printChannelMappings();
  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    uint8_t ch = keyMaps[i].ch;
    bool initialized = false;
    Serial.print("Init CH"); Serial.println(ch);
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_RETRIES; attempt++) {
      tcaSelect(ch);
      if (lox[i].begin()) {
        initialized = true;
        loxReady[i] = true;
        break;
      }
      delay(100);
    }
    if (!initialized) {
      loxReady[i] = false;
      Serial.print("WARNING: Sensor init failed CH");
      Serial.print(ch);
      Serial.print(" after ");
      Serial.print(SENSOR_INIT_RETRIES);
      Serial.println(" attempts");
      continue;
    }
    lox[i].setMeasurementTimingBudgetMicroSeconds(MEASUREMENT_BUDGET_US);
  }
  calibrateSensors();
  initAirTrixxWireless();
  sendAirTrixxKeyboardBattery(true);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  handleSerialCommand();
  if (airTrixxRecalibrationRequested) {
    airTrixxRecalibrationRequested = false;
    Serial.println("[KEYBOARD] ESP-NOW recalibration requested");
    calibrateSensors();
  }
  sendAirTrixxKeyboardBattery(false);

  int coveredCount=0, closeCount=0, farCount=0;
  int distPerCh[NUM_KEY_MAPS], relativePerCh[NUM_KEY_MAPS], rawPerCh[NUM_KEY_MAPS];
  bool rawValid[NUM_KEY_MAPS];

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    uint8_t ch = keyMaps[i].ch;
    int rawMM; distPerCh[i] = -1; relativePerCh[i] = -1;
    rawValid[i] = readRaw(ch, rawMM);
    rawPerCh[i] = rawValid[i] ? rawMM : -1;
    if (rawValid[i]) addMovingSample(ch, rawMM);
  }

  for (uint8_t i = 0; i < NUM_KEY_MAPS; i++) {
    uint8_t ch = keyMaps[i].ch;
    int avgRawMM;
    if (!getMovingAverage(ch, avgRawMM)) continue;
    int d = avgRawMM;
    relativePerCh[i] = d;
    if (d > maxSignalMM[i]) continue;
    if (d <= CAPS_DISTANCE_MM) closeCount++; else farCount++;
    if (!isRealCover(ch, avgRawMM)) continue;
    distPerCh[i] = d;
    coveredCount++;
  }

  sendAirTrixxKeyboardTof(rawPerCh, rawValid);

  if (loggingMode) {
    if (!logHeaderPrinted) { printLogHeader(); logHeaderPrinted = true; }
    if (millis() - lastLogMs >= LOG_SAMPLE_INTERVAL_MS) { printLogRow(rawPerCh, rawValid, relativePerCh); lastLogMs = millis(); }
    return;
  }

  if (coveredCount == 0 && prevCoveredCount > 0) {
    finishSwipeBuffer(nullptr);
    resetKeyState();
  }
  prevCoveredCount = coveredCount;

  if (coveredCount == NUM_KEY_MAPS && closeCount == NUM_KEY_MAPS) {
    if (capsStartMs == 0) capsStartMs = millis();
    if (!capsGestureLatched && millis() - capsStartMs >= CAPS_HOLD_MS) {
      capsMode = !capsMode; capsGestureLatched = true;
      resetMovingAverages(); resetKeyState();
      Serial.println(capsMode ? "CAPS ON" : "caps off");
    }
    numStartMs = 0; numGestureLatched = false; return;
  }
  capsStartMs = 0; capsGestureLatched = false;

  if (coveredCount == NUM_KEY_MAPS && farCount == NUM_KEY_MAPS) {
    if (millis() - lastNumToggleMs >= NUM_MODE_COOLDOWN_MS && numStartMs == 0) numStartMs = millis();
    if (!numGestureLatched && millis() - numStartMs >= NUM_HOLD_MS && millis() - lastNumToggleMs >= NUM_MODE_COOLDOWN_MS) {
      numbersMode = !numbersMode; numGestureLatched = true; lastNumToggleMs = millis();
      resetMovingAverages(); resetKeyState();
      Serial.println(numbersMode ? "NUMBERS MODE ON" : "NUMBERS MODE OFF");
    }
    return;
  }
  numStartMs = 0;
  if (millis() - lastNumToggleMs >= NUM_MODE_COOLDOWN_MS) numGestureLatched = false;

  int dominantIdx = findDominantSensor(distPerCh, rawValid);
  if (dominantIdx < 0) return;

  uint8_t dominantCh   = keyMaps[dominantIdx].ch;
  int     dominantDist = distPerCh[dominantIdx];

  char action = getActionForChannel(dominantCh, dominantDist);
  if (action == '\0') { resetKeyState(); return; }
  if (action != pendingKey) { pendingKey = action; pendingStartMs = millis(); }
  if (millis() - pendingStartMs >= KEY_DWELL_MS && action != lastPrintedKey) {
    handleKeyAction(action, dominantCh, dominantDist);
    lastPrintedKey = action;
    lastKeypressMs = millis();
  }
}
