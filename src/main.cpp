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
const unsigned long PRESENCE_TIMEOUT_MS = 45000;  // 45s

// ================== DISPLAY SH1106 SPI ==================
static const int PIN_SCK  = 18;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 5;
static const int PIN_DC   = 16;
static const int PIN_RST  = 4;

// ================== ESP-NOW CONFIG ==================
// MAC C3: 0C:4E:A0:30:37:20
uint8_t C3_MAC[] = { 0x0C, 0x4E, 0xA0, 0x30, 0x37, 0x20 };

// ================== AUTO CHANNEL (RTC) ==================
RTC_DATA_ATTR int8_t lockedChannel = -1;   // 1..13, -1 = non ancora trovato

// ===== HELLO packet (C3 -> broadcast) per discovery canale =====
typedef struct __attribute__((packed)) {
  uint8_t type;   // 2 = hello
  uint8_t ch;     // canale WiFi del C3
  uint32_t ms;
} HelloPacket;

static const uint8_t HELLO_TYPE = 2;

volatile bool gotHello = false;
volatile uint8_t helloCh = 0;

// ================== BATTERY CONFIG ==================
const float BATT_V_MIN = 3.30f;
const float BATT_V_MAX = 4.20f;
const int   ADC_BITS   = 12;

const float BATT_R1  = 100000.0f;
const float BATT_R2  = 100000.0f;
const float BATT_DIV = (BATT_R1 + BATT_R2) / BATT_R2;

const int   BATT_SAMPLES  = 32;
const float BATT_ALPHA    = 0.95f;
const int   BATT_HYST_PCT = 2;

// ================== MODALITA' RISPARMIO ==================
static const uint32_t NORMAL_WAKE_SEC = 0.5 * 60;   // 1 minuti
static const uint32_t LIVE_WAKE_SEC   = 30;         // 30 secondi
static const uint32_t LISTEN_WINDOW_MS_NORMAL = 20000; // ascolto comandi appena sveglio (normale)
static const uint32_t LISTEN_WINDOW_MS_LIVE   = 6000;  // ascolto comandi (live)

// Live duration: 10 minuti -> 10*60/30 = 20 cicli
static const uint16_t LIVE_DEFAULT_SEC = 10 * 60;

RTC_DATA_ATTR uint16_t liveCyclesRemaining = 0;  // persiste in deep sleep
RTC_DATA_ATTR uint8_t  relayState1 = 0;
RTC_DATA_ATTR uint8_t  relayState2 = 0;
RTC_DATA_ATTR uint8_t  relayState3 = 0;

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

TelemetryPacket pkt;

// ================== PACKET COMANDI (C3 -> slave) ==================
typedef struct __attribute__((packed)) {
  uint8_t type;      // 1 = comando
  uint8_t r1;        // 0/1 oppure 255 = ignorare
  uint8_t r2;        // 0/1 oppure 255 = ignorare
  uint8_t r3;        // 0/1 oppure 255 = ignorare
  uint8_t irrig;     // 1 = mostra IRRIGA (3s)
  uint16_t liveSec;  // 0 = ignora, altrimenti entra in LIVE per X secondi
  uint32_t ms;
} CommandPacket;

static const uint8_t CMD_TYPE = 1;
static const uint8_t CMD_KEEP = 255;

// ================== OGGETTI ==================
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, PIN_CS, PIN_DC, PIN_RST);
DHT dht(DHTPIN, DHTTYPE);

// ================== STATO SENSORI ==================
float temperature = NAN;
float humidity    = NAN;
int   soilValue   = 0;
int   soilPercent = 0;
int   batteryPercent = 0;

// Presenza/display
unsigned long lastPresenceTime = 0;
bool displayOn = true;

// UI "IRRIGA"
bool showIrrigazione = false;
unsigned long irrigazioneStart = 0;

// Occhi
int leftEyeX = 40, rightEyeX = 90, eyeY = 18, eyeWidth = 35, eyeHeight = 30;
int targetOffsetX = 0, targetOffsetY = 0, moveSpeed = 5;
int blinkState = 0;
unsigned long blinkDelay = 4000;
unsigned long lastBlinkTime = 0, moveTime = 0;

// ================== UTILITY DISPLAY ==================
void showSplash(const char* line1, const char* line2 = "", int ms = 0) {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  if (line1 && line1[0] != '\0') display.drawStr(2, 14, line1);
  if (line2 && line2[0] != '\0') display.drawStr(2, 30, line2);
  display.sendBuffer();
  if (ms > 0) delay(ms);
}

void drawEye(int x, int y, int w, int h) {
  display.drawRBox(x, y, w, h, 5);
}

void resetDisplayHardware() {
  pinMode(PIN_CS,  OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DC,  OUTPUT); digitalWrite(PIN_DC, HIGH);
  pinMode(PIN_RST, OUTPUT); digitalWrite(PIN_RST, HIGH);
  delay(60);
  digitalWrite(PIN_RST, LOW);  delay(12);
  digitalWrite(PIN_RST, HIGH); delay(60);
}

bool initDisplayRobusto(uint8_t tentativi = 5) {
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  for (uint8_t i = 1; i <= tentativi; i++) {
    resetDisplayHardware();
    display.begin();
    display.setPowerSave(0);
    display.setContrast(200);
    display.setBusClock(1000000UL);
    return true;
  }
  return false;
}

// ================== RELAY / PRESENZA ==================
void setRelay(uint8_t relayPin, bool on) {
  digitalWrite(relayPin, on ? HIGH : LOW); // active HIGH
}
bool readPresence() {
  int sig = digitalRead(PRESENCE_PIN);
  return PRESENCE_ACTIVE_HIGH ? (sig == HIGH) : (sig == LOW);
}

// ================== BATTERY ==================
static int readBatteryPercent(float* outVBattFilt=nullptr, float* outVBattRaw=nullptr, float* outVAdc=nullptr) {
  uint32_t sum_mv = 0;
  analogReadMilliVolts(BATT_PIN);
  for (int i = 0; i < BATT_SAMPLES; i++) {
    sum_mv += analogReadMilliVolts(BATT_PIN);
    delayMicroseconds(200);
  }

  float vAdc  = (sum_mv / (float)BATT_SAMPLES) / 1000.0f;
  float vBattRaw = vAdc * BATT_DIV;

  static float vBattFilt = 0.0f;
  if (vBattFilt == 0.0f) vBattFilt = vBattRaw;
  else vBattFilt = BATT_ALPHA * vBattFilt + (1.0f - BATT_ALPHA) * vBattRaw;

  if (outVAdc)      *outVAdc = vAdc;
  if (outVBattRaw)  *outVBattRaw = vBattRaw;
  if (outVBattFilt) *outVBattFilt = vBattFilt;

  float pct = (vBattFilt - BATT_V_MIN) * 100.0f / (BATT_V_MAX - BATT_V_MIN);
  pct = constrain(pct, 0.0f, 100.0f);
  int pctInt = (int)(pct + 0.5f);

  static int lastStable = -1;
  if (lastStable < 0) lastStable = pctInt;

  if (abs(pctInt - lastStable) < BATT_HYST_PCT) pctInt = lastStable;
  else lastStable = pctInt;

  return pctInt;
}

// ================== SENSORI ==================
void updateSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) {
    temperature = t;
    humidity    = h;
  }

  soilValue   = analogRead(SOIL_PIN);
  soilPercent = map(soilValue, 3000, 1200, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);

  batteryPercent = readBatteryPercent();
}

// ================== UI ==================
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
    } else {
      showIrrigazione = false;
    }
  }

  if (now - lastBlinkTime > blinkDelay && blinkState == 0) {
    blinkState = 1; lastBlinkTime = now;
  } else if (blinkState == 1 && now - lastBlinkTime > 150) {
    blinkState = 0; lastBlinkTime = now;
  }

  if (blinkState == 0 && now - moveTime > (unsigned long)random(1500, 3000)) {
    int m = random(0, 8);
    switch (m) {
      case 0: targetOffsetX=-10; targetOffsetY=0; break;
      case 1: targetOffsetX= 10; targetOffsetY=0; break;
      case 2: targetOffsetX=-10; targetOffsetY=-8; break;
      case 3: targetOffsetX= 10; targetOffsetY=-8; break;
      case 4: targetOffsetX=-10; targetOffsetY= 8; break;
      case 5: targetOffsetX= 10; targetOffsetY= 8; break;
      default: targetOffsetX=0; targetOffsetY=0; break;
    }
    moveTime = now;
  }

  offsetX += (targetOffsetX - offsetX) / moveSpeed;
  offsetY += (targetOffsetY - offsetY) / moveSpeed;

  display.clearBuffer();

  if (blinkState == 0) {
    drawEye(leftEyeX + offsetX,  eyeY + offsetY, eyeWidth, eyeHeight);
    drawEye(rightEyeX + offsetX, eyeY + offsetY, eyeWidth, eyeHeight);
  } else {
    display.drawBox(leftEyeX + offsetX,  eyeY + offsetY + eyeHeight/2 - 2, eyeWidth, 4);
    display.drawBox(rightEyeX + offsetX, eyeY + offsetY + eyeHeight/2 - 2, eyeWidth, 4);
  }

  char line[24];
  display.setFont(u8g2_font_5x8_tr);
  snprintf(line, sizeof(line), "T: %.1fC", temperature); display.drawStr(0, 10, line);
  snprintf(line, sizeof(line), "H: %.1f%%", humidity);    display.drawStr(0, 20, line);
  snprintf(line, sizeof(line), "S: %d%%", soilPercent);   display.drawStr(0, 30, line);
  snprintf(line, sizeof(line), "B: %d%%", batteryPercent);display.drawStr(0, 40, line);

  display.sendBuffer();
}

void handlePresence() {
  unsigned long now = millis();
  bool presence = readPresence();
  if (presence) {
    lastPresenceTime = now;
    if (!displayOn) { displayOn = true; display.setPowerSave(0); }
  } else {
    if (displayOn && (now - lastPresenceTime >= PRESENCE_TIMEOUT_MS)) {
      displayOn = false; display.setPowerSave(1);
    }
  }

  // 2) PowerStatePacket
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

    char ev[128];
    snprintf(ev, sizeof(ev),
      "{\"type\":\"power_state\",\"relayMask\":%u,\"resetReason\":%u}",
      (unsigned)st.relayMask, (unsigned)st.resetReason
    );
    mqtt.publish(PWR_EVENT, ev, false);

    // ✅ aggiornamento immediato del LAST quando cambiano i relè (se abbiamo telemetria valida)
    if (haveTelemetry) publishLastToMqtt(viewPkt);

    return;
  }

  (void)mac;
}

// ================== ESP-NOW ==================
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
volatile bool sendDone = false;

void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
  lastSendStatus = status;
  sendDone = true;
  Serial.print("ESP-NOW sent -> ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// RX comandi + HELLO discovery
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // 1) HELLO (broadcast dal C3): serve solo per scoprire il canale
  if (len == (int)sizeof(HelloPacket)) {
    HelloPacket h;
    memcpy(&h, data, sizeof(h));
    if (h.type == HELLO_TYPE) {
      gotHello = true;
      helloCh = h.ch;
      return;
    }
  }

  // 2) CommandPacket (solo dal MAC del C3)
  if (memcmp(mac, C3_MAC, 6) != 0) return;
  if (len != (int)sizeof(CommandPacket)) return;

  CommandPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.type != CMD_TYPE) return;

  if (cmd.r1 != CMD_KEEP) { relayState1 = (cmd.r1 == 1); setRelay(RELAY1, relayState1); }
  if (cmd.r2 != CMD_KEEP) { relayState2 = (cmd.r2 == 1); setRelay(RELAY2, relayState2); }
  if (cmd.r3 != CMD_KEEP) { relayState3 = (cmd.r3 == 1); setRelay(RELAY3, relayState3); }

  if (cmd.irrig == 1) { showIrrigazione = true; irrigazioneStart = millis(); }

  if (cmd.liveSec > 0) {
    uint16_t cycles = (uint16_t)((cmd.liveSec + LIVE_WAKE_SEC - 1) / LIVE_WAKE_SEC);
    liveCyclesRemaining = cycles;
    Serial.print("LIVE requested sec="); Serial.print(cmd.liveSec);
    Serial.print(" -> cycles="); Serial.println(liveCyclesRemaining);
  }
}

static bool initEspNowOnChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  delay(20);

  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE); // evita power save

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL");
    return false;
  }

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  // peer unicast (C3) per telemetria con ACK
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, C3_MAC, 6);
  peer.channel = ch;
  peer.encrypt = false;

  esp_now_del_peer(C3_MAC);
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW add_peer FAIL");
    return false;
  }

  Serial.print("ESP-NOW OK ch="); Serial.println(ch);
  Serial.print("SENSORS MAC: "); Serial.println(WiFi.macAddress());
  return true;
}

// Scan canali 1..13 fino a ricevere HELLO dal C3
static bool findChannelFromHello(uint32_t maxMs = 7000) {
  gotHello = false;

  uint32_t start = millis();
  while (millis() - start < maxMs) {
    for (uint8_t ch = 1; ch <= 13; ch++) {
      // ascolto su canale ch
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);

      uint32_t t0 = millis();
      while (millis() - t0 < 240) { // finestra ascolto
        if (gotHello) {
          lockedChannel = (int8_t)helloCh;
          Serial.print("LOCKED CHANNEL from HELLO = ");
          Serial.println(lockedChannel);
          return true;
        }
        delay(5);
      }
    }
  }
  return false;
}

bool sendToC3WithRetry(uint8_t tries = 3) {
  pkt.t = temperature;
  pkt.h = humidity;
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

// ================== DEEP SLEEP CONTROL ==================
void goToSleep(bool liveMode) {
  uint32_t sleepSec = liveMode ? LIVE_WAKE_SEC : NORMAL_WAKE_SEC;

  Serial.print("SLEEP "); Serial.print(sleepSec);
  Serial.print("s (liveCyclesRemaining="); Serial.print(liveCyclesRemaining);
  Serial.println(")");

  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32 SENSORI (SLAVE) AVVIATO");

  randomSeed((uint32_t)esp_random());

  // Display
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  bool ok = initDisplayRobusto();
  if (!ok) {
    resetDisplayHardware();
    display.begin();
    display.setPowerSave(0);
    display.setContrast(200);
    display.setBusClock(1000000UL);
  }
  showSplash("Display OK", "Slave avvio...", 250);

  // DHT
  dht.begin();

  // ADC
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);

  // Relè (ripristino stato da RTC)
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  setRelay(RELAY1, relayState1 == 1);
  setRelay(RELAY2, relayState2 == 1);
  setRelay(RELAY3, relayState3 == 1);

  // PIR
  pinMode(PRESENCE_PIN, INPUT);
  lastPresenceTime = millis();
  displayOn = true;
  display.setPowerSave(0);

  // ==== ESP-NOW con auto-canale ====
  // 1) prima inizializzo ESPNOW su un canale "placeholder" (1) per poter ricevere callback HELLO durante lo scan
  if (!initEspNowOnChannel(1)) {
    showSplash("ESP-NOW", "Init FAIL", 800);
  }

  // 2) se non abbiamo un canale lockato, facciamo scan per trovare HELLO dal C3
  if (lockedChannel < 1 || lockedChannel > 13) {
    showSplash("AUTO CH", "Scanning...", 0);
    bool found = findChannelFromHello(7000);
    if (!found) {
      Serial.println("AUTO CH: NOT FOUND (fallback ch=1)");
      lockedChannel = 1;
    }
  } else {
    Serial.print("AUTO CH: using lockedChannel=");
    Serial.println(lockedChannel);
  }

  // 3) re-init ESPNOW sul canale lockato
  esp_now_deinit();
  delay(20);
  initEspNowOnChannel((uint8_t)lockedChannel);

  // ==== finestra ascolto comandi ====
  bool liveMode = (liveCyclesRemaining > 0);
  uint32_t listenWindow = liveMode ? LISTEN_WINDOW_MS_LIVE : LISTEN_WINDOW_MS_NORMAL;

  Serial.print("MODE: "); Serial.println(liveMode ? "LIVE" : "NORMAL");

  uint32_t t0 = millis();
  while (millis() - t0 < listenWindow) {
    delay(10);
  }

  // ==== aggiorna sensori + invia ====
  updateSensors();
  bool txOk = sendToC3WithRetry(3);
  Serial.println(txOk ? "TX OK" : "TX FAIL");

  // UI minima
  handlePresence();
  if (displayOn) {
    drawUI();
    delay(120);
  }

  // Se siamo in LIVE, consumiamo 1 ciclo
  if (liveCyclesRemaining > 0) liveCyclesRemaining--;

  // Sleep
  goToSleep(liveCyclesRemaining > 0);
}

void loop() {
  // Non ci arriviamo: usiamo deep sleep
}
