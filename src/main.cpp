#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>

#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <LittleFS.h>
#include <Preferences.h>

#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// ===== WIFI / MQTT =====
const char* ssid     = "H6645P-75235224_2.4GHz";
const char* password = "zQ3Y7Q4RDt";

const char* mqtt_server = "345aa86858ae4b6d8df7851ca6cfafbf.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "EVEAndroid01";
const char* mqtt_pass   = "Abracadabra@2025";

const char* TOPIC_HOURLY = "progetto/EVE/hourly";
const char* TOPIC_LAST   = "progetto/EVE/last";

// ===== POWER TOPICS =====
const char* PWR_RELAY_SET[4] = {
  "progetto/EVE/POWER/relay/1/set",
  "progetto/EVE/POWER/relay/2/set",
  "progetto/EVE/POWER/relay/3/set",
  "progetto/EVE/POWER/relay/4/set"
};

const char* PWR_RELAY_STATE[4] = {
  "progetto/EVE/POWER/relay/1/state",
  "progetto/EVE/POWER/relay/2/state",
  "progetto/EVE/POWER/relay/3/state",
  "progetto/EVE/POWER/relay/4/state"
};

// schedule topics (separati)
const char* PWR_RELAY_SCHED_SET[4] = {
  "progetto/EVE/POWER/relay/1/schedule/set",
  "progetto/EVE/POWER/relay/2/schedule/set",
  "progetto/EVE/POWER/relay/3/schedule/set",
  "progetto/EVE/POWER/relay/4/schedule/set"
};

const char* PWR_RELAY_SCHED_STATE[4] = {
  "progetto/EVE/POWER/relay/1/schedule",
  "progetto/EVE/POWER/relay/2/schedule",
  "progetto/EVE/POWER/relay/3/schedule",
  "progetto/EVE/POWER/relay/4/schedule"
};

const char* PWR_STATE = "progetto/EVE/POWER/state";
const char* PWR_EVENT = "progetto/EVE/POWER/event";

// ===== MQTT =====
WiFiClientSecure net;
PubSubClient mqtt(net);

// ===== PIN TFT =====
#define PIN_SCK  4
#define PIN_MOSI 6
#define PIN_CS   7
#define PIN_DC   10
#define PIN_RST  1

Adafruit_GC9A01A tft(PIN_CS, PIN_DC, PIN_RST);
GFXcanvas16 canvas(240, 240);

// ===== COLORI =====
#define BLACK 0x0000
#define WHITE 0xFFFF
#define BLUE  0x001F

// ===== ESP-NOW CONFIG =====
static const uint8_t ESPNOW_CHANNEL = 1;
uint8_t BCAST_MAC[] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

// ===== HELLO packet (Master -> broadcast) =====
typedef struct __attribute__((packed)) {
  uint8_t type;   // 2 = hello
  uint8_t ch;     // canale WiFi corrente
  uint32_t ms;
} HelloPacket;
static const uint8_t HELLO_TYPE = 2;

// ===== POWER PACKETS =====
static const uint8_t PWR_CMD_TYPE       = 10;
static const uint8_t PWR_STATE_TYPE     = 12;
static const uint8_t PWR_TIME_TYPE      = 13; // minute-only
static const uint8_t PWR_RELAYRULE_TYPE = 14; // rules per relay (max 10)

typedef struct __attribute__((packed)) {
  uint8_t type;     // 10
  uint8_t maskSet;  // bit0..3
  uint8_t maskVal;  // bit0..3
  uint8_t applyNow; // compat
  uint32_t ms;
} PowerCmdPacket;

typedef struct __attribute__((packed)) {
  uint16_t minuteOfDay; // 0..1439
  uint8_t  on;          // 1=ON 0=OFF
  uint8_t  daysMask;    // bit0..6 lun..dom
} RelayRuleBin;

typedef struct __attribute__((packed)) {
  uint8_t type;     // 14
  uint8_t ch;       // 1..4
  uint8_t count;    // 0..10
  RelayRuleBin rules[10];
  uint32_t ms;
} PowerRelayRulesPacket;

typedef struct __attribute__((packed)) {
  uint8_t type;       // 12
  uint8_t relayMask;  // bit0..3
  uint8_t timeValid;  // 0/1
  uint8_t resetReason;// esp_reset_reason()
  uint32_t ms;
} PowerStatePacket;

typedef struct __attribute__((packed)) {
  uint8_t type;         // 13
  uint16_t minuteOfDay; // 0..1439
  uint8_t weekdayMon0;  // 0=lun..6=dom
  uint8_t valid;        // 0/1
  uint32_t ms;
} PowerTimePacket;

// ===== PACKET SENSORS =====
typedef struct __attribute__((packed)) {
  float t;
  float h;
  uint8_t soil;
  uint8_t batt;
  uint8_t r1, r2, r3;
  uint8_t presence;
  uint32_t ms;
} TelemetryPacket;

volatile bool hasPkt = false;
TelemetryPacket rx;
TelemetryPacket viewPkt;
unsigned long lastPktAt = 0;
bool haveTelemetry = false;

// ===== POWER STATE CACHE =====
volatile uint8_t powerRelayMask = 0;
static inline uint8_t pwrGet(uint8_t ch1to4) { return (powerRelayMask >> (ch1to4-1)) & 0x01; }

volatile uint32_t rxCount = 0;

// ===== OCCHI =====
struct Eye { float cx, cy; float w, h; };
Eye L = { 80, 120, 70, 95 };
Eye R = {160, 120, 70, 95 };

float lookX = 0, lookY = 0;
float targetX = 0, targetY = 0;

bool blinking = false;
uint32_t blinkStart = 0;
uint32_t nextBlink = 0;

float ease(float t) { return t * t * (3 - 2 * t); }
void scheduleBlink() { nextBlink = millis() + random(2000, 5000); }

void drawEye(const Eye& e, float lx, float ly, float blinkAmt) {
  float open = 1.0f - blinkAmt;
  float ew = e.w;
  float eh = e.h * open;

  canvas.fillRoundRect((int)(e.cx - ew/2), (int)(e.cy - eh/2), (int)ew, (int)eh, (int)(ew * 0.5f), BLUE);

  float px = e.cx + lx * 18;
  float py = e.cy + ly * 12;

  canvas.fillRoundRect((int)(px - 18), (int)(py - 22), 36, 44, 18, BLACK);
}

// ===== STORAGE (LittleFS) =====
static const char* RAW_PATH  = "/raw.jsonl";
static const char* HOUR_PATH = "/hour.jsonl";

const size_t MAX_RAW_BYTES  = 500 * 1024;
const size_t MAX_HOUR_BYTES = 200 * 1024;

bool rotateIfTooBig(const char* path, const char* oldPath, size_t maxBytes) {
  File f = LittleFS.open(path, "a+");
  if (!f) return false;
  size_t sz = f.size();
  f.close();
  if (sz <= maxBytes) return true;

  if (LittleFS.exists(oldPath)) LittleFS.remove(oldPath);
  if (LittleFS.exists(path)) LittleFS.rename(path, oldPath);

  File nf = LittleFS.open(path, "w");
  if (!nf) return false;
  nf.close();
  return true;
}

bool appendLine(const char* path, const char* line) {
  File f = LittleFS.open(path, "a");
  if (!f) return false;
  size_t w = f.print(line);
  f.close();
  return w > 0;
}

bool saveRaw(const TelemetryPacket& p) {
  rotateIfTooBig(RAW_PATH, "/raw.old", MAX_RAW_BYTES);

  char buf[280];
  snprintf(buf, sizeof(buf),
           "{\"ms\":%lu,\"t\":%.2f,\"h\":%.2f,\"soil\":%u,\"batt\":%u,"
           "\"r1\":%u,\"r2\":%u,\"r3\":%u,\"r4\":%u,\"pir\":%u}\n",
           (unsigned long)p.ms, p.t, p.h, p.soil, p.batt,
           pwrGet(1), pwrGet(2), pwrGet(3), pwrGet(4),
           p.presence);

  return appendLine(RAW_PATH, buf);
}

// ===== AGGREGAZIONE ORARIA =====
struct HourAgg {
  bool active = false;
  uint32_t hourIdx = 0;
  double sumT = 0, sumH = 0, sumSoil = 0, sumBatt = 0;
  uint16_t n = 0;
} agg;

uint32_t hourIndexFromMs(uint32_t ms) { return ms / 3600000UL; }

bool flushHourAggToFile(uint32_t hourIdx, double tAvg, double hAvg, double sAvg, double bAvg, uint16_t n) {
  rotateIfTooBig(HOUR_PATH, "/hour.old", MAX_HOUR_BYTES);

  char buf[240];
  snprintf(buf, sizeof(buf),
           "{\"hour_idx\":%lu,\"t_avg\":%.2f,\"h_avg\":%.2f,\"soil_avg\":%.2f,\"batt_avg\":%.2f,\"n\":%u}\n",
           (unsigned long)hourIdx, tAvg, hAvg, sAvg, bAvg, n);

  return appendLine(HOUR_PATH, buf);
}

void publishHourlyToMqtt(uint32_t hourIdx, double tAvg, double hAvg, double sAvg, double bAvg, uint16_t n) {
  if (!mqtt.connected()) return;

  char payload[220];
  snprintf(payload, sizeof(payload),
           "{\"hour_idx\":%lu,\"t_avg\":%.2f,\"h_avg\":%.2f,\"soil_avg\":%.2f,\"batt_avg\":%.2f,\"n\":%u}",
           (unsigned long)hourIdx, tAvg, hAvg, sAvg, bAvg, n);

  mqtt.publish(TOPIC_HOURLY, payload, false);
}

void publishLastToMqtt(const TelemetryPacket& p) {
  if (!mqtt.connected()) return;

  char payload[280];
  snprintf(payload, sizeof(payload),
           "{\"ms\":%lu,\"t\":%.2f,\"h\":%.2f,\"soil\":%u,\"batt\":%u,"
           "\"r1\":%u,\"r2\":%u,\"r3\":%u,\"r4\":%u,\"pir\":%u}",
           (unsigned long)p.ms, p.t, p.h, p.soil, p.batt,
           pwrGet(1), pwrGet(2), pwrGet(3), pwrGet(4),
           p.presence);

  mqtt.publish(TOPIC_LAST, payload, true);
}

void updateHourAgg(const TelemetryPacket& p) {
  uint32_t hIdx = hourIndexFromMs(p.ms);

  if (!agg.active) { agg.active = true; agg.hourIdx = hIdx; }

  if (hIdx != agg.hourIdx) {
    if (agg.n > 0) {
      double tAvg = agg.sumT / agg.n;
      double hAvg = agg.sumH / agg.n;
      double sAvg = agg.sumSoil / agg.n;
      double bAvg = agg.sumBatt / agg.n;

      flushHourAggToFile(agg.hourIdx, tAvg, hAvg, sAvg, bAvg, agg.n);
      publishHourlyToMqtt(agg.hourIdx, tAvg, hAvg, sAvg, bAvg, agg.n);
    }

    agg.hourIdx = hIdx;
    agg.sumT = agg.sumH = agg.sumSoil = agg.sumBatt = 0;
    agg.n = 0;
  }

  agg.sumT += p.t;
  agg.sumH += p.h;
  agg.sumSoil += p.soil;
  agg.sumBatt += p.batt;
  agg.n++;
}

// ===== UTIL TIME/DAYS =====
static uint16_t hhmmToMin(const char* s) {
  int hh=0, mm=0;
  if (sscanf(s, "%d:%d", &hh, &mm) != 2) return 0;
  hh = constrain(hh, 0, 23);
  mm = constrain(mm, 0, 59);
  return (uint16_t)(hh*60 + mm);
}

static uint8_t parseDaysMask(const char* daysS) {
  // daysS: "1111100" (lun..dom)
  uint8_t dm = 0;
  if (!daysS) return 0b01111111;
  for (int i=0; i<7 && daysS[i]; i++) {
    if (daysS[i] == '1') dm |= (1 << i);
  }
  if (dm == 0) dm = 0b01111111;
  return dm;
}

static uint8_t weekdayMon0FromTmWday(int tm_wday) {
  if (tm_wday == 0) return 6;      // dom -> 6
  return (uint8_t)(tm_wday - 1);   // lun -> 0
}

// ===== MASTER PERSIST (rules JSON per relay) =====
Preferences mprefs;
static const char* MASTER_NS = "evemaster";
static String keyRelayRules(uint8_t ch1to4) { return String("rr") + String((int)ch1to4); }
static void saveRelayRulesJson(uint8_t ch1to4, const String& rawJson) {
  mprefs.putString(keyRelayRules(ch1to4).c_str(), rawJson);
}
static String loadRelayRulesJson(uint8_t ch1to4) {
  return mprefs.getString(keyRelayRules(ch1to4).c_str(), "");
}

// ===== Send rules packet to POWER =====
static bool buildRulesPacketFromJson(uint8_t ch1to4, const String& json, PowerRelayRulesPacket& outPkt) {
  StaticJsonDocument<2048> doc;
  auto err = deserializeJson(doc, json);
  if (err || !doc.is<JsonArray>()) return false;

  outPkt.type = PWR_RELAYRULE_TYPE;
  outPkt.ch = ch1to4;
  outPkt.count = 0;
  outPkt.ms = millis();
  memset(outPkt.rules, 0, sizeof(outPkt.rules));

  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject o : arr) {
    if (outPkt.count >= 10) break;

    const char* atS   = o["at"]    | nullptr;  // "HH:MM"
    const char* stS   = o["state"] | nullptr;  // "ON"/"OFF"
    const char* daysS = o["days"]  | "1111111";

    if (!atS || !stS) continue;

    String st(stS);
    st.trim(); st.toUpperCase();
    if (st != "ON" && st != "OFF") continue;

    RelayRuleBin r;
    r.minuteOfDay = hhmmToMin(atS);
    r.on = (st == "ON") ? 1 : 0;
    r.daysMask = parseDaysMask(daysS);

    outPkt.rules[outPkt.count++] = r;
  }

  return true; // count può essere 0 (svuota)
}

static void sendRulesToPower(uint8_t ch1to4, const PowerRelayRulesPacket& pkt) {
  esp_now_send(BCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

static void publishScheduleFeedback(uint8_t ch1to4, const char* status, uint8_t count) {
  if (!mqtt.connected()) return;
  char fb[96];
  snprintf(fb, sizeof(fb), "{\"status\":\"%s\",\"count\":%u}", status, (unsigned)count);
  mqtt.publish(PWR_RELAY_SCHED_STATE[ch1to4-1], fb, true); // retained JSON
}

// ===== MQTT CALLBACK =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String t(topic);

  // 1) Manual relay set: ON/OFF/TOGGLE
  for (int ch = 1; ch <= 4; ch++) {
    if (t == String(PWR_RELAY_SET[ch-1])) {
      PowerCmdPacket pc;
      pc.type = PWR_CMD_TYPE;
      pc.maskSet = (1 << (ch-1));
      pc.maskVal = 0;
      pc.applyNow = 0;
      pc.ms = millis();

      bool curOn = ((powerRelayMask >> (ch-1)) & 0x01) != 0;

      if (msg.equalsIgnoreCase("ON")) pc.maskVal = (1 << (ch-1));
      else if (msg.equalsIgnoreCase("OFF")) pc.maskVal = 0;
      else if (msg.equalsIgnoreCase("TOGGLE")) pc.maskVal = (curOn ? 0 : (1 << (ch-1)));
      else pc.maskVal = 0;

      esp_now_send(BCAST_MAC, (uint8_t*)&pc, sizeof(pc));
      mqtt.publish(PWR_EVENT, "{\"type\":\"power_cmd\",\"status\":\"sent\"}", false);
      return;
    }
  }

  // 2) Relay schedule set: JSON array (max 10)
  for (int ch = 1; ch <= 4; ch++) {
    if (t == String(PWR_RELAY_SCHED_SET[ch-1])) {
      PowerRelayRulesPacket rp;
      if (!buildRulesPacketFromJson((uint8_t)ch, msg, rp)) {
        publishScheduleFeedback((uint8_t)ch, "INVALID_JSON", 0);
        mqtt.publish(PWR_EVENT, "{\"type\":\"relay_schedule\",\"status\":\"invalid_json\"}", false);
        return;
      }

      // salva raw json sul master
      saveRelayRulesJson((uint8_t)ch, msg);

      // invia al power
      sendRulesToPower((uint8_t)ch, rp);

      // feedback retained JSON
      publishScheduleFeedback((uint8_t)ch, "OK", rp.count);

      char ev[140];
      snprintf(ev, sizeof(ev),
        "{\"type\":\"relay_schedule\",\"ch\":%d,\"count\":%u,\"status\":\"saved_and_sent\"}",
        ch, (unsigned)rp.count
      );
      mqtt.publish(PWR_EVENT, ev, false);

      return;
    }
  }
}

// ===== ESP-NOW RX =====
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  rxCount++;
  (void)info;

  // Telemetry
  if (len == (int)sizeof(TelemetryPacket)) {
    memcpy((void*)&rx, data, sizeof(TelemetryPacket));
    hasPkt = true;
    lastPktAt = millis();
    return;
  }

  // Power state
  if (len == (int)sizeof(PowerStatePacket)) {
    PowerStatePacket st;
    memcpy(&st, data, sizeof(st));
    if (st.type != PWR_STATE_TYPE) return;

    powerRelayMask = st.relayMask;

    for (int i=0; i<4; i++) {
      bool on = (st.relayMask >> i) & 0x01;
      mqtt.publish(PWR_RELAY_STATE[i], on ? "ON" : "OFF", true);
    }

    char buf[128];
    snprintf(buf, sizeof(buf),
      "{\"relayMask\":%u,\"timeValid\":%u,\"resetReason\":%u}",
      (unsigned)st.relayMask, (unsigned)st.timeValid, (unsigned)st.resetReason
    );
    mqtt.publish(PWR_STATE, buf, true);

    if (haveTelemetry) publishLastToMqtt(viewPkt);
    return;
  }
}

bool initEspNowRx() {
  WiFi.mode(WIFI_STA);
  delay(50);

  esp_wifi_start();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_del_peer(BCAST_MAC);
  esp_now_add_peer(&peer);

  return true;
}

// ===== UI =====
void drawOverlay(const TelemetryPacket& p, bool linkOk) {
  canvas.setTextColor(WHITE);
  canvas.setTextWrap(false);

  canvas.setTextSize(2);
  canvas.setCursor(10, 10);
  canvas.print("EVE");

  canvas.setTextSize(1);
  canvas.setCursor(10, 30);
  canvas.print("ESP-NOW: ");
  canvas.print(linkOk ? "OK" : "NO DATA");

  canvas.setCursor(10, 40);
  canvas.print("RX: ");
  canvas.print((unsigned long)rxCount);

  canvas.setCursor(10, 50);
  canvas.print("MQTT: ");
  canvas.print(mqtt.connected() ? "OK" : "OFF");

  canvas.setTextSize(2);

  canvas.setCursor(10, 70);
  canvas.printf("T %.1fC", p.t);

  canvas.setCursor(10, 95);
  canvas.printf("H %d%%", (int)(p.h + 0.5f));

  canvas.setCursor(10, 120);
  canvas.printf("S %d%%", p.soil);

  canvas.setCursor(10, 145);
  canvas.printf("B %d%%", p.batt);

  canvas.setTextSize(1);
  canvas.setCursor(10, 170);
  canvas.printf("PWR R1:%u R2:%u R3:%u R4:%u PIR:%u",
                pwrGet(1), pwrGet(2), pwrGet(3), pwrGet(4),
                p.presence);
}

// ===== MQTT =====
static void publishRetainedSchedulesFeedbackFromNVS() {
  for (int ch=1; ch<=4; ch++) {
    String j = loadRelayRulesJson((uint8_t)ch);
    if (j.length() > 0) publishScheduleFeedback((uint8_t)ch, "OK", 255); // 255 = "unknown count" on boot
  }
}

static void resendSavedSchedulesToPower() {
  for (int ch=1; ch<=4; ch++) {
    String j = loadRelayRulesJson((uint8_t)ch);
    if (j.length() == 0) continue;

    PowerRelayRulesPacket rp;
    if (!buildRulesPacketFromJson((uint8_t)ch, j, rp)) continue;
    sendRulesToPower((uint8_t)ch, rp);
  }
}

void mqttConnect() {
  if (mqtt.connected()) return;

  net.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  String clientId = "EVE-C3-" + WiFi.macAddress();

  if (mqtt.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    for (int i=0; i<4; i++) mqtt.subscribe(PWR_RELAY_SET[i]);
    for (int i=0; i<4; i++) mqtt.subscribe(PWR_RELAY_SCHED_SET[i]);

    publishRetainedSchedulesFeedbackFromNVS();
    mqtt.publish(PWR_EVENT, "{\"type\":\"mqtt\",\"status\":\"connected\"}", false);
  }
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(200);
  }
}

static void setupTimeNtp() {
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
}

static void sendHello() {
  HelloPacket h;
  h.type = HELLO_TYPE;
  h.ch   = WiFi.channel();
  h.ms   = millis();
  esp_now_send(BCAST_MAC, (uint8_t*)&h, sizeof(h));
}

// ================== SETUP / LOOP ==================
void setup() {
  randomSeed(esp_random());

  Serial.begin(115200);
  delay(800);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount FAILED");
  mprefs.begin(MASTER_NS, false);

  wifiConnect();
  setupTimeNtp();

  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  tft.begin();
  tft.setRotation(0);

  canvas.fillScreen(BLACK);
  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 240);

  if (!initEspNowRx()) Serial.println("ESP-NOW init FAIL");
  else Serial.println("ESP-NOW RX OK");

  // reinvia al POWER le sched salvate
  resendSavedSchedulesToPower();

  mqttConnect();

  memset((void*)&viewPkt, 0, sizeof(viewPkt));
  viewPkt.t = NAN;
  viewPkt.h = NAN;
  scheduleBlink();
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttConnect();
    mqtt.loop();
  }

  // TIME SYNC to POWER every 30s (minute only)
  static uint32_t lastTimeSync = 0;
  if (WiFi.status() == WL_CONNECTED && (now - lastTimeSync > 30000)) {
    lastTimeSync = now;

    tm tinfo;
    if (getLocalTime(&tinfo, 50)) {
      PowerTimePacket tp;
      tp.type = PWR_TIME_TYPE;
      tp.minuteOfDay = (uint16_t)(tinfo.tm_hour * 60 + tinfo.tm_min);
      tp.weekdayMon0 = weekdayMon0FromTmWday(tinfo.tm_wday);
      tp.valid = 1;
      tp.ms = millis();
      esp_now_send(BCAST_MAC, (uint8_t*)&tp, sizeof(tp));
    }
  }

  // HELLO every 1s
  static uint32_t lastHello = 0;
  if (now - lastHello > 1000) {
    lastHello = now;
    sendHello();
  }

  // Telemetria sensori
  if (hasPkt) {
    noInterrupts();
    viewPkt = rx;
    hasPkt = false;
    interrupts();

    haveTelemetry = true;

    saveRaw(viewPkt);
    updateHourAgg(viewPkt);
    publishLastToMqtt(viewPkt);
  }

  // PUBBLICAZIONE FORZATA OGNI 60s
  static uint32_t lastForcedLast = 0;
  if (haveTelemetry && mqtt.connected() && (now - lastForcedLast >= 60000UL)) {
    lastForcedLast = now;
    publishLastToMqtt(viewPkt);
  }

  // animazione occhi
  if (random(0, 100) < 2) {
    targetX = random(-10, 11) / 10.0f;
    targetY = random(-6, 7) / 10.0f;
  }
  lookX += (targetX - lookX) * 0.12f;
  lookY += (targetY - lookY) * 0.12f;

  float blinkAmt = 0.0f;
  if (!blinking && now > nextBlink) { blinking = true; blinkStart = now; }
  if (blinking) {
    float t = (now - blinkStart) / 180.0f;
    if (t >= 1.0f) { blinking = false; scheduleBlink(); }
    else blinkAmt = ease(t < 0.5 ? t*2 : (1 - t)*2);
  }

  canvas.fillScreen(BLACK);
  drawEye(L, lookX, lookY, blinkAmt);
  drawEye(R, lookX, lookY, blinkAmt);

  bool linkOk = (now - lastPktAt) <= 5000;
  drawOverlay(viewPkt, linkOk);

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 240);

  delay(16);
}
