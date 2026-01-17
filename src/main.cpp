#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <esp_now.h>
#include <esp_wifi.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// ================== CONFIG ==================
// WiFi
const char* ssid     = "H6645P-75235224_2.4GHz";
const char* password = "zQ3Y7Q4RDt";

// MQTT (HiveMQ Cloud)
const char* mqtt_server = "345aa86858ae4b6d8df7851ca6cfafbf.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "EVEAndroid01";
const char* mqtt_pass   = "Abracadabra@2025";

// Topic (uguali al tuo ESP32 sensori)
// Topic
const char* TOPIC_T       = "progetto/EVE/temperatura";
const char* TOPIC_H       = "progetto/EVE/umidita";
const char* TOPIC_S       = "progetto/EVE/suolo";
const char* TOPIC_BATT    = "progetto/EVE/batteria";
const char* TOPIC_CMD     = "progetto/EVE/irrigazione";
const char* TOPIC_STATUS  = "progetto/EVE/esp/status";
const char* TOPIC_RELE1   = "progetto/EVE/rele1";
const char* TOPIC_RELE2   = "progetto/EVE/rele2";
const char* TOPIC_RELE3   = "progetto/EVE/rele3";

// ================== TFT (GC9A01A) ==================
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

// ================== ESP-NOW CONFIG ==================
static const uint8_t ESPNOW_CHANNEL = 1;

// ================== PACKET TELEMETRIA ==================
typedef struct __attribute__((packed)) {
  float t;
  float h;
  uint8_t soil;
  uint8_t batt;
  uint8_t r1, r2, r3;
  uint8_t presence;
  uint32_t ms;
} TelemetryPacket;

// ================== PACKET COMANDI (C3 -> slave) ==================
typedef struct __attribute__((packed)) {
  uint8_t type;      // 1 = comando
  uint8_t r1;        // 0/1 oppure 255 = ignorare
  uint8_t r2;        // 0/1 oppure 255 = ignorare
  uint8_t r3;        // 0/1 oppure 255 = ignorare
  uint8_t irrig;     // 1 = mostra IRRIGA (3s)
  uint32_t ms;
} CommandPacket;

static const uint8_t CMD_TYPE = 1;
static const uint8_t CMD_KEEP = 255;

// broadcast per inviare comandi (lo slave filtra per MAC del C3)
uint8_t BCAST_MAC[] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

// ================== MQTT ==================
WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);

// ================== RX STATE ==================
volatile bool hasPkt = false;
TelemetryPacket rx;
TelemetryPacket viewPkt;
unsigned long lastPktAt = 0;
volatile uint32_t rxCount = 0;

// ================== OCCHI UI ==================
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

  canvas.fillRoundRect(
    (int)(e.cx - ew/2),
    (int)(e.cy - eh/2),
    (int)ew,
    (int)eh,
    (int)(ew * 0.5f),
    BLUE
  );

  float px = e.cx + lx * 18;
  float py = e.cy + ly * 12;

  canvas.fillRoundRect(
    (int)(px - 18),
    (int)(py - 22),
    36,
    44,
    18,
    BLACK
  );
}

String makeClientId() {
  char id[40];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(id, sizeof(id), "C3-%06X", (uint32_t)(mac & 0xFFFFFF));
  return String(id);
}

// ================== ESP-NOW RX CALLBACK ==================
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  rxCount++;

  if (len == (int)sizeof(TelemetryPacket)) {
    memcpy((void*)&rx, data, sizeof(TelemetryPacket));
    hasPkt = true;
    lastPktAt = millis();
  }
}

// ================== ESP-NOW INIT ==================
bool initEspNow() {
  // WiFi stack deve essere up
  esp_wifi_start();

  // forza canale fisso
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);

  // peer broadcast per inviare comandi
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  return true;
}

void sendCommandToSlave(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t irrig) {
  CommandPacket cmd;
  cmd.type = CMD_TYPE;
  cmd.r1 = r1;
  cmd.r2 = r2;
  cmd.r3 = r3;
  cmd.irrig = irrig;
  cmd.ms = millis();

  // invio broadcast: lo slave accetta SOLO se mac mittente == MAC del C3
  esp_now_send(BCAST_MAC, (uint8_t*)&cmd, sizeof(cmd));
}

// ================== MQTT ==================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String t(topic);

  if (t == TOPIC_CMD && msg == "ON") {
    // comando irrigazione (solo overlay; se vuoi anche relè, fallo lato slave)
    sendCommandToSlave(CMD_KEEP, CMD_KEEP, CMD_KEEP, 1);
    return;
  }

  if (t == TOPIC_RELE1) sendCommandToSlave(msg == "ON" ? 1 : 0, CMD_KEEP, CMD_KEEP, 0);
  else if (t == TOPIC_RELE2) sendCommandToSlave(CMD_KEEP, msg == "ON" ? 1 : 0, CMD_KEEP, 0);
  else if (t == TOPIC_RELE3) sendCommandToSlave(CMD_KEEP, CMD_KEEP, msg == "ON" ? 1 : 0, 0);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("WiFi: ");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " KO");
}

void connectMQTTNonBloccante() {
  static unsigned long lastAttempt = 0;
  static uint8_t backoff = 0;

  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastAttempt < (1000UL << backoff)) return;

  lastAttempt = now;
  if (backoff < 5) backoff++;

  String cid = makeClientId();
  Serial.print("MQTT connect... ");

  bool ok = mqtt.connect(
    cid.c_str(),
    mqtt_user, mqtt_pass,
    TOPIC_STATUS, 1, true, "offline", true
  );

  if (ok) {
    Serial.println("OK");
    backoff = 0;

    mqtt.subscribe(TOPIC_CMD);
    mqtt.subscribe(TOPIC_RELE1);
    mqtt.subscribe(TOPIC_RELE2);
    mqtt.subscribe(TOPIC_RELE3);

    mqtt.publish(TOPIC_STATUS, "online", true);
  } else {
    Serial.print("KO rc=");
    Serial.println(mqtt.state());
  }
}

// ================== PUBLISH TELEMETRY ==================
void publishTelemetry(const TelemetryPacket& p) {
  if (!mqtt.connected()) return;

  char buf[32];

  dtostrf(p.t, 0, 1, buf);
  mqtt.publish(TOPIC_T, buf, true);

  dtostrf(p.h, 0, 1, buf);
  mqtt.publish(TOPIC_H, buf, true);

  snprintf(buf, sizeof(buf), "%d", (int)p.soil);
  mqtt.publish(TOPIC_S, buf, true);

  snprintf(buf, sizeof(buf), "%d", (int)p.batt);
  mqtt.publish(TOPIC_BATT, buf, true);
}

// ================== UI OVERLAY ==================
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
  canvas.print("MQTT: ");
  canvas.print(mqtt.connected() ? "OK" : "KO");

  canvas.setCursor(10, 50);
  canvas.print("RX: ");
  canvas.print((unsigned long)rxCount);

  if (WiFi.status() == WL_CONNECTED) {
    canvas.setCursor(10, 60);
    canvas.print("RSSI: ");
    canvas.print(WiFi.RSSI());
  }

  canvas.setTextSize(2);

  canvas.setCursor(10, 85);
  canvas.printf("T %.1fC", p.t);

  canvas.setCursor(10, 110);
  canvas.printf("H %d%%", (int)(p.h + 0.5f));

  canvas.setCursor(10, 135);
  canvas.printf("S %d%%", p.soil);

  canvas.setCursor(10, 160);
  canvas.printf("B %d%%", p.batt);

  canvas.setTextSize(1);
  canvas.setCursor(10, 185);
  canvas.printf("R1:%d R2:%d R3:%d PIR:%d", p.r1, p.r2, p.r3, p.presence);
}

// ================== SETUP / LOOP ==================
void setup() {
  randomSeed(esp_random());

  Serial.begin(115200);
  delay(1200);

  Serial.print("C3 MAC: ");
  Serial.println(WiFi.macAddress());

  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  tft.begin();
  tft.setRotation(0);

  canvas.fillScreen(BLACK);
  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 240);

  // WiFi + MQTT
  connectWiFi();
  wifiClient.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  // ESP-NOW (canale fisso 1)
  if (!initEspNow()) {
    Serial.println("ESP-NOW init FAIL");
  } else {
    Serial.print("ESP-NOW RX OK (ch=");
    Serial.print(ESPNOW_CHANNEL);
    Serial.println(")");
  }

  memset((void*)&viewPkt, 0, sizeof(viewPkt));
  viewPkt.t = NAN;
  viewPkt.h = NAN;

  scheduleBlink();
}

void loop() {
  uint32_t now = millis();

  // MQTT non bloccante
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) connectMQTTNonBloccante();
  mqtt.loop();

  // copia pacchetto ricevuto
  if (hasPkt) {
    noInterrupts();
    viewPkt = rx;
    hasPkt = false;
    interrupts();

    // publish appena arriva (replica comportamento vecchio)
    publishTelemetry(viewPkt);
  }

  // animazioni occhi
  if (random(0, 100) < 2) {
    targetX = random(-10, 11) / 10.0f;
    targetY = random(-6, 7) / 10.0f;
  }

  lookX += (targetX - lookX) * 0.12f;
  lookY += (targetY - lookY) * 0.12f;

  float blinkAmt = 0.0f;
  if (!blinking && now > nextBlink) {
    blinking = true;
    blinkStart = now;
  }
  if (blinking) {
    float t = (now - blinkStart) / 180.0f;
    if (t >= 1.0f) {
      blinking = false;
      scheduleBlink();
    } else {
      blinkAmt = ease(t < 0.5 ? t*2 : (1 - t)*2);
    }
  }

  canvas.fillScreen(BLACK);
  drawEye(L, lookX, lookY, blinkAmt);
  drawEye(R, lookX, lookY, blinkAmt);

  bool linkOk = (now - lastPktAt) <= 5000;
  drawOverlay(viewPkt, linkOk);

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 240);

  delay(16);
}
