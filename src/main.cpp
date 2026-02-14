#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <DHT.h>
#include <SPI.h>
#include <U8g2lib.h>

// ================== PIN ==================
#define DHTPIN   19
#define DHTTYPE  DHT22
#define SOIL_PIN 32
#define BATT_PIN 34
#define RELAY1   25
#define RELAY2   26
#define RELAY3   27
#define PRESENCE_PIN 33

const bool PRESENCE_ACTIVE_HIGH = true;
const unsigned long PRESENCE_TIMEOUT_MS = 45000;

// ================== DISPLAY SH1106 SPI ==================
static const int PIN_SCK  = 18;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 5;
static const int PIN_DC   = 16;
static const int PIN_RST  = 4;

// ================== ESP-NOW CONFIG ==================
uint8_t C3_MAC[] = { 0x0C, 0x4E, 0xA0, 0x30, 0x37, 0x20 };
RTC_DATA_ATTR int8_t lockedChannel = -1;

// ================== ENERGY ==================
static const uint32_t NORMAL_WAKE_SEC = 30;
static const uint32_t LIVE_WAKE_SEC   = 30;
static const uint32_t LISTEN_WINDOW_MS_NORMAL = 20000;
static const uint32_t LISTEN_WINDOW_MS_LIVE   = 6000;
static const uint16_t LIVE_DEFAULT_SEC = 10 * 60;

RTC_DATA_ATTR uint16_t liveCyclesRemaining = 0;
RTC_DATA_ATTR uint8_t  relayState1 = 0;
RTC_DATA_ATTR uint8_t  relayState2 = 0;
RTC_DATA_ATTR uint8_t  relayState3 = 0;

// ================== BATTERY ==================
const float BATT_V_MIN = 3.30f;
const float BATT_V_MAX = 4.20f;
const int   ADC_BITS   = 12;
const float BATT_R1  = 100000.0f;
const float BATT_R2  = 100000.0f;
const float BATT_DIV = (BATT_R1 + BATT_R2) / BATT_R2;
const int   BATT_SAMPLES  = 32;
const float BATT_ALPHA    = 0.95f;
const int   BATT_HYST_PCT = 2;

// ================== PACKETS ==================
static const uint8_t HELLO_TYPE = 2;
static const uint8_t CMD_TYPE = 1;
static const uint8_t CMD_KEEP = 255;
static const uint8_t SCHED_SYNC_TYPE = 3;
static const uint8_t SCHED_ACK_TYPE = 4;
static const uint8_t SCHED_EXECUTED_TYPE = 5;
static const uint8_t TIME_SYNC_TYPE = 6;

static const uint8_t MAX_RELAYS = 3;
static const uint8_t MAX_SCHEDULE_RULES = 10;

#pragma pack(push,1)
typedef struct {
  uint8_t type;
  uint8_t ch;
  uint32_t ms;
} HelloPacket;

typedef struct {
  float t;
  float h;
  uint8_t soil;
  uint8_t batt;
  uint8_t r1, r2, r3;
  uint8_t presence;
  uint32_t ms;
} TelemetryPacket;

typedef struct {
  uint8_t type;
  uint8_t r1;
  uint8_t r2;
  uint8_t r3;
  uint8_t irrig;
  uint16_t liveSec;
  uint32_t ms;
} CommandPacket;

typedef struct {
  uint8_t hh;
  uint8_t mm;
  uint8_t state;
  uint8_t daysMask; // bit0 lun ... bit6 dom
} ScheduleRule;

typedef struct {
  uint8_t type;
  uint8_t relayIdx; // 1..3
  uint8_t count;    // 0..10
  ScheduleRule rules[MAX_SCHEDULE_RULES];
  uint32_t ms;
} ScheduleSyncPacket;

typedef struct {
  uint8_t type;
  uint8_t relayIdx;
  uint8_t result; // 1 ok, 0 fail
  uint8_t count;
  uint32_t ms;
} ScheduleAckPacket;

typedef struct {
  uint8_t type;
  uint8_t relayIdx;
  uint8_t state;
  uint8_t ruleIndex;
  uint8_t weekdayMon0;
  uint8_t hh;
  uint8_t mm;
  uint32_t ms;
} ScheduleExecutedPacket;

typedef struct {
  uint8_t type;
  uint16_t minuteOfDay;
  uint8_t weekdayMon0;
  uint8_t valid;
  uint32_t ms;
} TimeSyncPacket;
#pragma pack(pop)

// ================== RTC SCHEDULE STATE ==================
RTC_DATA_ATTR ScheduleRule relaySchedules[MAX_RELAYS][MAX_SCHEDULE_RULES];
RTC_DATA_ATTR uint8_t relayScheduleCount[MAX_RELAYS] = {0, 0, 0};
RTC_DATA_ATTR uint16_t lastExecutedMinute[MAX_RELAYS] = {65535, 65535, 65535};
RTC_DATA_ATTR uint8_t  lastExecutedWday[MAX_RELAYS] = {255, 255, 255};

RTC_DATA_ATTR uint16_t currentMinuteOfDay = 0;
RTC_DATA_ATTR uint8_t currentWeekdayMon0 = 0;
RTC_DATA_ATTR bool timeValid = false;

volatile bool gotHello = false;
volatile uint8_t helloCh = 0;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
volatile bool sendDone = false;

// ================== GLOBALS ==================
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, PIN_CS, PIN_DC, PIN_RST);
DHT dht(DHTPIN, DHTTYPE);
TelemetryPacket pkt;

float temperature = NAN;
float humidity    = NAN;
int soilValue = 0;
int soilPercent = 0;
int batteryPercent = 0;

unsigned long lastPresenceTime = 0;
bool displayOn = true;
bool showIrrigazione = false;
unsigned long irrigazioneStart = 0;

int leftEyeX = 40, rightEyeX = 90, eyeY = 18, eyeWidth = 35, eyeHeight = 30;
int targetOffsetX = 0, targetOffsetY = 0, moveSpeed = 5;
int blinkState = 0;
unsigned long blinkDelay = 4000;
unsigned long lastBlinkTime = 0, moveTime = 0;

void setRelay(uint8_t relayPin, bool on) { digitalWrite(relayPin, on ? HIGH : LOW); }
bool readPresence() { int sig = digitalRead(PRESENCE_PIN); return PRESENCE_ACTIVE_HIGH ? (sig == HIGH) : (sig == LOW); }

void setRelayByIndex(uint8_t relayIdx, bool on) {
  switch (relayIdx) {
    case 1: relayState1 = on; setRelay(RELAY1, on); break;
    case 2: relayState2 = on; setRelay(RELAY2, on); break;
    case 3: relayState3 = on; setRelay(RELAY3, on); break;
    default: break;
  }
}

void showSplash(const char* line1, const char* line2 = "", int ms = 0) {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  if (line1 && line1[0] != '\0') display.drawStr(2, 14, line1);
  if (line2 && line2[0] != '\0') display.drawStr(2, 30, line2);
  display.sendBuffer();
  if (ms > 0) delay(ms);
}

static int readBatteryPercent() {
  uint32_t sum_mv = 0;
  analogReadMilliVolts(BATT_PIN); // priming
  for (int i = 0; i < BATT_SAMPLES; i++) {
    sum_mv += analogReadMilliVolts(BATT_PIN);
    delayMicroseconds(200);
  }
  float vAdc = (sum_mv / (float)BATT_SAMPLES) / 1000.0f;
  float vBattRaw = vAdc * BATT_DIV;
  static float vBattFilt = 0.0f;
  if (vBattFilt == 0.0f) vBattFilt = vBattRaw;
  else vBattFilt = BATT_ALPHA * vBattFilt + (1.0f - BATT_ALPHA) * vBattRaw;
  float pct = (vBattFilt - BATT_V_MIN) * 100.0f / (BATT_V_MAX - BATT_V_MIN);
  pct = constrain(pct, 0.0f, 100.0f);
  int pctInt = (int)(pct + 0.5f);
  static int lastStable = -1;
  if (lastStable < 0) lastStable = pctInt;
  if (abs(pctInt - lastStable) < BATT_HYST_PCT) pctInt = lastStable;
  else lastStable = pctInt;
  return pctInt;
}

void updateSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) { temperature = t; humidity = h; }
  soilValue = analogRead(SOIL_PIN);
  soilPercent = constrain(map(soilValue, 3000, 1200, 0, 100), 0, 100);
  batteryPercent = readBatteryPercent();
}

void drawUI() {
  static int offsetX = 0, offsetY = 0;
  unsigned long now = millis();

  if (showIrrigazione) {
    if (now - irrigazioneStart < 3000) {
      display.clearBuffer();
      display.setFont(u8g2_font_ncenB14_tr);
      display.setCursor(10, 35);
      display.print("IRRIGA");
      display.sendBuffer();
      return;
    }
    showIrrigazione = false;
  }

  if (now - lastBlinkTime > blinkDelay && blinkState == 0) { blinkState = 1; lastBlinkTime = now; }
  else if (blinkState == 1 && now - lastBlinkTime > 150) { blinkState = 0; lastBlinkTime = now; }

  if (blinkState == 0 && now - moveTime > (unsigned long)random(1500, 3000)) {
    int m = random(0, 8);
    switch (m) {
      case 0: targetOffsetX = -10; targetOffsetY = 0; break;
      case 1: targetOffsetX = 10; targetOffsetY = 0; break;
      case 2: targetOffsetX = -10; targetOffsetY = -8; break;
      case 3: targetOffsetX = 10; targetOffsetY = -8; break;
      case 4: targetOffsetX = -10; targetOffsetY = 8; break;
      case 5: targetOffsetX = 10; targetOffsetY = 8; break;
      default: targetOffsetX = 0; targetOffsetY = 0; break;
    }
    moveTime = now;
  }

  offsetX += (targetOffsetX - offsetX) / moveSpeed;
  offsetY += (targetOffsetY - offsetY) / moveSpeed;

  display.clearBuffer();
  if (blinkState == 0) {
    display.drawRBox(leftEyeX + offsetX, eyeY + offsetY, eyeWidth, eyeHeight, 5);
    display.drawRBox(rightEyeX + offsetX, eyeY + offsetY, eyeWidth, eyeHeight, 5);
  } else {
    display.drawBox(leftEyeX + offsetX, eyeY + offsetY + eyeHeight/2 - 2, eyeWidth, 4);
    display.drawBox(rightEyeX + offsetX, eyeY + offsetY + eyeHeight/2 - 2, eyeWidth, 4);
  }

  char line[24];
  display.setFont(u8g2_font_5x8_tr);
  snprintf(line, sizeof(line), "T: %.1fC", temperature); display.drawStr(0, 10, line);
  snprintf(line, sizeof(line), "H: %.1f%%", humidity); display.drawStr(0, 20, line);
  snprintf(line, sizeof(line), "S: %d%%", soilPercent); display.drawStr(0, 30, line);
  snprintf(line, sizeof(line), "B: %d%%", batteryPercent); display.drawStr(0, 40, line);
  display.sendBuffer();
}

void handlePresence() {
  unsigned long now = millis();
  if (readPresence()) {
    lastPresenceTime = now;
    if (!displayOn) { displayOn = true; display.setPowerSave(0); }
  } else if (displayOn && (now - lastPresenceTime >= PRESENCE_TIMEOUT_MS)) {
    displayOn = false;
    display.setPowerSave(1);
  }
}

void onEspNowSent(const uint8_t*, esp_now_send_status_t status) { lastSendStatus = status; sendDone = true; }

void sendScheduleAck(uint8_t relayIdx, bool ok, uint8_t count) {
  ScheduleAckPacket ack{};
  ack.type = SCHED_ACK_TYPE;
  ack.relayIdx = relayIdx;
  ack.result = ok ? 1 : 0;
  ack.count = count;
  ack.ms = millis();
  esp_now_send(C3_MAC, (uint8_t*)&ack, sizeof(ack));
}

void sendScheduleExecuted(uint8_t relayIdx, uint8_t state, uint8_t ruleIndex) {
  ScheduleExecutedPacket ev{};
  ev.type = SCHED_EXECUTED_TYPE;
  ev.relayIdx = relayIdx;
  ev.state = state;
  ev.ruleIndex = ruleIndex;
  ev.weekdayMon0 = currentWeekdayMon0;
  ev.hh = currentMinuteOfDay / 60;
  ev.mm = currentMinuteOfDay % 60;
  ev.ms = millis();
  esp_now_send(C3_MAC, (uint8_t*)&ev, sizeof(ev));
}

void applySchedulesIfDue() {
  if (!timeValid) return;
  for (uint8_t r = 0; r < MAX_RELAYS; r++) {
    uint8_t cnt = relayScheduleCount[r];
    for (uint8_t i = 0; i < cnt; i++) {
      ScheduleRule &rule = relaySchedules[r][i];
      if (!(rule.daysMask & (1 << currentWeekdayMon0))) continue;
      uint16_t m = (uint16_t)rule.hh * 60 + rule.mm;
      if (m != currentMinuteOfDay) continue;
      if (lastExecutedMinute[r] == m && lastExecutedWday[r] == currentWeekdayMon0) continue;

      bool on = rule.state == 1;
      setRelayByIndex(r + 1, on);
      sendScheduleExecuted(r + 1, on ? 1 : 0, i);
      lastExecutedMinute[r] = m;
      lastExecutedWday[r] = currentWeekdayMon0;
    }
  }
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len >= 2) {
    HelloPacket h{};
    size_t helloLen = (size_t)len < sizeof(h) ? (size_t)len : sizeof(h);
    memcpy(&h, data, helloLen);
    if (h.type == HELLO_TYPE && h.ch >= 1 && h.ch <= 13) {
      gotHello = true;
      helloCh = h.ch;
      return;
    }
  }
  if (memcmp(mac, C3_MAC, 6) != 0 || len < 1) return;

  uint8_t type = data[0];
  if (type == CMD_TYPE && len >= 5) {
    CommandPacket cmd{};
    size_t cmdLen = (size_t)len < sizeof(cmd) ? (size_t)len : sizeof(cmd);
    memcpy(&cmd, data, cmdLen);
    if (cmd.r1 != CMD_KEEP) setRelayByIndex(1, cmd.r1 == 1);
    if (cmd.r2 != CMD_KEEP) setRelayByIndex(2, cmd.r2 == 1);
    if (cmd.r3 != CMD_KEEP) setRelayByIndex(3, cmd.r3 == 1);
    if (cmd.irrig == 1) { showIrrigazione = true; irrigazioneStart = millis(); }
    if (cmd.liveSec > 0) {
      uint16_t cycles = (uint16_t)((cmd.liveSec + LIVE_WAKE_SEC - 1) / LIVE_WAKE_SEC);
      liveCyclesRemaining = cycles;
    }
    return;
  }

  if (type == SCHED_SYNC_TYPE && len >= 7) {
    ScheduleSyncPacket sync{};
    size_t syncLen = (size_t)len < sizeof(sync) ? (size_t)len : sizeof(sync);
    memcpy(&sync, data, syncLen);

    bool ok = true;
    if (sync.relayIdx < 1 || sync.relayIdx > MAX_RELAYS || sync.count > MAX_SCHEDULE_RULES) ok = false;
    size_t expectedMinLen = 3 + ((size_t)sync.count * sizeof(ScheduleRule));
    if ((size_t)len < expectedMinLen) ok = false;
    if (!ok) { sendScheduleAck(sync.relayIdx, false, 0); return; }

    uint8_t relayPos = sync.relayIdx - 1;
    relayScheduleCount[relayPos] = sync.count;
    for (uint8_t i = 0; i < sync.count; i++) relaySchedules[relayPos][i] = sync.rules[i];
    sendScheduleAck(sync.relayIdx, true, sync.count);
    return;
  }

  if (type == TIME_SYNC_TYPE && len >= 5) {
    TimeSyncPacket ts{};
    size_t tsLen = (size_t)len < sizeof(ts) ? (size_t)len : sizeof(ts);
    memcpy(&ts, data, tsLen);
    if (ts.valid == 1 && ts.weekdayMon0 < 7 && ts.minuteOfDay < 1440) {
      timeValid = true;
      currentMinuteOfDay = ts.minuteOfDay;
      currentWeekdayMon0 = ts.weekdayMon0;
      applySchedulesIfDue();
    }
  }
}

static bool initEspNowOnChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  delay(20);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, C3_MAC, 6);
  peer.channel = ch;
  peer.encrypt = false;
  esp_now_del_peer(C3_MAC);
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

static bool findChannelFromHello(uint32_t maxMs = 7000) {
  gotHello = false;
  uint32_t start = millis();
  while (millis() - start < maxMs) {
    for (uint8_t ch = 1; ch <= 13; ch++) {
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      uint32_t t0 = millis();
      while (millis() - t0 < 240) {
        if (gotHello) { lockedChannel = (int8_t)helloCh; return true; }
        delay(5);
      }
    }
  }
  return false;
}

bool sendToC3WithRetry(uint8_t tries = 3) {
  pkt.t = temperature; pkt.h = humidity;
  pkt.soil = (uint8_t)soilPercent;
  pkt.batt = (uint8_t)batteryPercent;
  pkt.r1 = digitalRead(RELAY1) ? 1 : 0;
  pkt.r2 = digitalRead(RELAY2) ? 1 : 0;
  pkt.r3 = digitalRead(RELAY3) ? 1 : 0;
  pkt.presence = readPresence() ? 1 : 0;
  pkt.ms = millis();

  for (uint8_t i = 0; i < tries; i++) {
    sendDone = false;
    esp_now_send(C3_MAC, (uint8_t*)&pkt, sizeof(pkt));
    uint32_t t0 = millis();
    while (!sendDone && millis() - t0 < 250) delay(1);
    if (sendDone && lastSendStatus == ESP_NOW_SEND_SUCCESS) return true;
    delay(35);
  }
  return false;
}

void goToSleep(bool liveMode) {
  uint32_t sleepSec = liveMode ? LIVE_WAKE_SEC : NORMAL_WAKE_SEC;
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed((uint32_t)esp_random());

  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  display.begin();
  display.setPowerSave(0);
  display.setContrast(200);
  display.setBusClock(1000000UL);
  showSplash("Slave", "Boot...", 150);

  dht.begin();
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);

  pinMode(RELAY1, OUTPUT); pinMode(RELAY2, OUTPUT); pinMode(RELAY3, OUTPUT);
  setRelay(RELAY1, relayState1 == 1);
  setRelay(RELAY2, relayState2 == 1);
  setRelay(RELAY3, relayState3 == 1);

  pinMode(PRESENCE_PIN, INPUT);
  lastPresenceTime = millis();
  displayOn = true;

  if (!initEspNowOnChannel(1)) showSplash("ESP-NOW", "Init FAIL", 500);
  if (lockedChannel < 1 || lockedChannel > 13) {
    if (!findChannelFromHello(7000)) lockedChannel = 1;
  }
  esp_now_deinit();
  delay(20);
  initEspNowOnChannel((uint8_t)lockedChannel);

  bool liveMode = (liveCyclesRemaining > 0);
  uint32_t listenWindow = liveMode ? LISTEN_WINDOW_MS_LIVE : LISTEN_WINDOW_MS_NORMAL;
  uint32_t t0 = millis();
  while (millis() - t0 < listenWindow) delay(10);

  updateSensors();
  sendToC3WithRetry(3);
  handlePresence();
  if (displayOn) { drawUI(); delay(120); }

  if (liveCyclesRemaining == 0 && !timeValid) {
    liveCyclesRemaining = (LIVE_DEFAULT_SEC + LIVE_WAKE_SEC - 1) / LIVE_WAKE_SEC;
  }
  if (liveCyclesRemaining > 0) liveCyclesRemaining--;

  goToSleep(liveCyclesRemaining > 0);
}

void loop() {}
