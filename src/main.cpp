#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <SPI.h>

// ================== CONFIG ==================
// WiFi
const char* ssid     = "H6645P-75235224_2.4GHz";
const char* password = "zQ3Y7Q4RDt";

// MQTT (HiveMQ Cloud)
const char* mqtt_server = "345aa86858ae4b6d8df7851ca6cfafbf.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "EVEAndroid01";
const char* mqtt_pass   = "Abracadabra@2025";

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

// Display SPI pins
static const int PIN_SCK  = 18;   // VSPI SCK
static const int PIN_MOSI = 23;   // VSPI MOSI
static const int PIN_CS   = 5;
static const int PIN_DC   = 16;
static const int PIN_RST  = 4;

// ================== PIN ==================
// Sensori
#define DHTPIN   19
#define DHTTYPE  DHT22
#define SOIL_PIN 32

// Batteria (partitore su GPIO34)
#define BATT_PIN 34

// Relè
#define RELAY1   25
#define RELAY2   26
#define RELAY3   27

// Presenza (SR602 PIR)
#define PRESENCE_PIN 33
const bool PRESENCE_ACTIVE_HIGH = true;
const unsigned long PRESENCE_TIMEOUT_MS = 45000;  // 45s

// ================== BATTERY CONFIG ==================
const float BATT_V_MIN = 3.30f;   // 0% (indicativo)
const float BATT_V_MAX = 4.20f;   // 100%
const int   ADC_BITS   = 12;

// Partitore: BAT+ -> R1 -> (nodo ADC) -> R2 -> GND
const float BATT_R1  = 100000.0f;  // ohm
const float BATT_R2  = 100000.0f;  // ohm
const float BATT_DIV = (BATT_R1 + BATT_R2) / BATT_R2;

// Stabilizzazione batteria
const int   BATT_SAMPLES = 32;    // campioni per lettura
const float BATT_ALPHA   = 0.95f; // filtro 0..1 (più alto = più stabile)
const int   BATT_HYST_PCT = 2;    // isteresi %: non cambia se differenza < 2 punti

// ================== OGGETTI ==================
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, /*cs=*/PIN_CS, /*dc=*/PIN_DC, /*reset=*/PIN_RST);
DHT dht(DHTPIN, DHTTYPE);

WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);

// ================== STATO ==================
float temperature = 0;
float humidity    = 0;
int   soilValue   = 0;
int   soilPercent = 0;
int   batteryPercent = 0;

bool showIrrigazione = false;
unsigned long irrigazioneStart = 0;

unsigned long lastTelemetryMs   = 0;
const unsigned long telemetryIntMs = 3000;

// Presenza/display
unsigned long lastPresenceTime = 0;
bool displayOn = true;

// Occhi
int leftEyeX = 40, rightEyeX = 90, eyeY = 18, eyeWidth = 35, eyeHeight = 30;
int targetOffsetX = 0, targetOffsetY = 0, moveSpeed = 5;
int blinkState = 0;
unsigned long blinkDelay = 4000;
unsigned long lastBlinkTime = 0, moveTime = 0;

// ================== UTILITY ==================
String makeClientId() {
  char id[40];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(id, sizeof(id), "ESP32-%06X", (uint32_t)(mac & 0xFFFFFF));
  return String(id);
}

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

// ================== DISPLAY FIRST ==================
void resetDisplayHardware() {
  pinMode(PIN_CS,  OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DC,  OUTPUT); digitalWrite(PIN_DC, HIGH);
  pinMode(PIN_RST, OUTPUT); digitalWrite(PIN_RST, HIGH);
  delay(60);

  digitalWrite(PIN_RST, LOW);  delay(12);
  digitalWrite(PIN_RST, HIGH); delay(60);
}

bool initDisplayRobusto(uint8_t tentativi = 5) {
  SPI.begin(PIN_SCK, /*MISO=*/-1, PIN_MOSI, PIN_CS);

  for (uint8_t i = 1; i <= tentativi; i++) {
    resetDisplayHardware();
    display.begin();
    display.setPowerSave(0);
    display.setContrast(200);
    display.setBusClock(1000000UL);

    showSplash("Display init...", "Tentativo", 0);

    display.clearBuffer();
    display.drawBox(0, 0, 10, 10);
    display.sendBuffer();
    delay(50);

    display.clearBuffer();
    display.setFont(u8g2_font_5x8_tr);
    display.drawStr(2, 10, "Display OK");
    display.sendBuffer();

    return true;
  }
  return false;
}

// ================== RELAY UTILS ==================
void setRelay(uint8_t relayPin, bool on) {
  digitalWrite(relayPin, on ? HIGH : LOW); // active HIGH
}

// ================== WIFI / MQTT ==================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long t0 = millis();
  showSplash("WiFi...", "connessione");
  Serial.print("WiFi: ");
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    display.clearBuffer();
    display.setFont(u8g2_font_5x8_tr);
    display.drawStr(2, 10, "WiFi OK");
    display.sendBuffer();
  } else {
    Serial.println(" KO");
    showSplash("WiFi KO", "timeout 15s", 800);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String t(topic);

  if (t == TOPIC_CMD && msg == "ON") {
    showIrrigazione = true;
    irrigazioneStart = millis();
  }

  if (t == TOPIC_RELE1)      setRelay(RELAY1, msg == "ON");
  else if (t == TOPIC_RELE2) setRelay(RELAY2, msg == "ON");
  else if (t == TOPIC_RELE3) setRelay(RELAY3, msg == "ON");
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
  showSplash("MQTT...", "connessione");

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

    display.clearBuffer();
    display.setFont(u8g2_font_5x8_tr);
    display.drawStr(2, 10, "MQTT OK");
    display.sendBuffer();
    delay(200);
  } else {
    Serial.print("KO rc=");
    Serial.println(mqtt.state());
    showSplash("MQTT KO", "retry...", 300);
  }
}

// ================== BATTERY READ (FILTRATA + ISTERESI) ==================
static int readBatteryPercent(float* outVBattFilt = nullptr, float* outVBattRaw = nullptr, float* outVAdc = nullptr) {
  // Media su ADC in mV
  uint32_t sum_mv = 0;
  analogReadMilliVolts(BATT_PIN); // scarto
  for (int i = 0; i < BATT_SAMPLES; i++) {
    sum_mv += analogReadMilliVolts(BATT_PIN);
    delayMicroseconds(200);
  }

  float vAdc  = (sum_mv / (float)BATT_SAMPLES) / 1000.0f;
  float vBattRaw = vAdc * BATT_DIV;

  // Filtro esponenziale sulla V batteria
  static float vBattFilt = 0.0f;
  if (vBattFilt == 0.0f) vBattFilt = vBattRaw;
  else vBattFilt = BATT_ALPHA * vBattFilt + (1.0f - BATT_ALPHA) * vBattRaw;

  if (outVAdc)      *outVAdc = vAdc;
  if (outVBattRaw)  *outVBattRaw = vBattRaw;
  if (outVBattFilt) *outVBattFilt = vBattFilt;

  // Percentuale da tensione filtrata
  float pct = (vBattFilt - BATT_V_MIN) * 100.0f / (BATT_V_MAX - BATT_V_MIN);
  pct = constrain(pct, 0.0f, 100.0f);
  int pctInt = (int)(pct + 0.5f);

  // Isteresi sulla percentuale (anti-sfarfallio)
  static int lastStable = -1;
  if (lastStable < 0) lastStable = pctInt;

  if (abs(pctInt - lastStable) < BATT_HYST_PCT) {
    pctInt = lastStable;
  } else {
    lastStable = pctInt;
  }

  return pctInt;
}

// ================== SENSORS & TELEMETRY ==================
void updateSensors() {
  // DHT22
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {
    temperature = t;
    humidity    = h;
  } else {
    Serial.println("ERRORE DHT22: lettura fallita");
  }

  // Suolo
  soilValue   = analogRead(SOIL_PIN);
  soilPercent = map(soilValue, 3000, 1200, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);

  // Batteria (filtrata + isteresi)
  float vBattFilt = 0, vBattRaw = 0, vAdc = 0;
  batteryPercent = readBatteryPercent(&vBattFilt, &vBattRaw, &vAdc);

  // Debug
  Serial.print("T="); Serial.print(temperature);
  Serial.print("C  H="); Serial.print(humidity);
  Serial.print("%  S="); Serial.print(soilPercent);
  Serial.print("%  Batt="); Serial.print(batteryPercent);
  Serial.print("%  (vRaw="); Serial.print(vBattRaw, 3);
  Serial.print("V vFilt="); Serial.print(vBattFilt, 3);
  Serial.print("V vAdc="); Serial.print(vAdc, 3);
  Serial.println("V)");
}

void publishTelemetry() {
  if (!mqtt.connected()) return;

  char buf[32];

  dtostrf(temperature, 0, 1, buf);
  mqtt.publish(TOPIC_T, buf, true);

  dtostrf(humidity, 0, 1, buf);
  mqtt.publish(TOPIC_H, buf, true);

  snprintf(buf, sizeof(buf), "%d", soilPercent);
  mqtt.publish(TOPIC_S, buf, true);

  snprintf(buf, sizeof(buf), "%d", batteryPercent);
  mqtt.publish(TOPIC_BATT, buf, true);
}

// ================== UI (OCCHI + TESTO) ==================
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
    blinkState = 1;
    lastBlinkTime = now;
  } else if (blinkState == 1 && now - lastBlinkTime > 150) {
    blinkState = 0;
    lastBlinkTime = now;
  }

  if (blinkState == 0 && now - moveTime > (unsigned long)random(1500, 3000)) {
    int m = random(0, 8);
    switch (m) {
      case 0: targetOffsetX = -10; targetOffsetY = 0;  break;
      case 1: targetOffsetX =  10; targetOffsetY = 0;  break;
      case 2: targetOffsetX = -10; targetOffsetY = -8; break;
      case 3: targetOffsetX =  10; targetOffsetY = -8; break;
      case 4: targetOffsetX = -10; targetOffsetY =  8; break;
      case 5: targetOffsetX =  10; targetOffsetY =  8; break;
      default: targetOffsetX =   0; targetOffsetY =  0; break;
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
    display.drawBox(leftEyeX + offsetX,  eyeY + offsetY + eyeHeight / 2 - 2, eyeWidth, 4);
    display.drawBox(rightEyeX + offsetX, eyeY + offsetY + eyeHeight / 2 - 2, eyeWidth, 4);
  }

  char line[24];
  display.setFont(u8g2_font_5x8_tr);

  snprintf(line, sizeof(line), "T: %.1fC", temperature);
  display.drawStr(0, 10, line);

  snprintf(line, sizeof(line), "H: %.1f%%", humidity);
  display.drawStr(0, 20, line);

  snprintf(line, sizeof(line), "S: %d%%", soilPercent);
  display.drawStr(0, 30, line);

  snprintf(line, sizeof(line), "B: %d%%", batteryPercent);
  display.drawStr(0, 40, line);

  display.sendBuffer();
}

// ================== PRESENCE HANDLING (SR602) ==================
void handlePresence() {
  unsigned long now = millis();
  int sig = digitalRead(PRESENCE_PIN);

  bool presence = PRESENCE_ACTIVE_HIGH ? (sig == HIGH) : (sig == LOW);

  if (presence) {
    lastPresenceTime = now;
    if (!displayOn) {
      Serial.println("TI VEDO → accendo display");
      displayOn = true;
      display.setPowerSave(0);
    }
  } else {
    if (displayOn && (now - lastPresenceTime >= PRESENCE_TIMEOUT_MS)) {
      Serial.println("non TI VEDO → spengo display");
      displayOn = false;
      display.setPowerSave(1);
    }
  }
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32 AVVIATO");

  SPI.begin(PIN_SCK, /*MISO=*/-1, PIN_MOSI, PIN_CS);
  bool ok = initDisplayRobusto();
  if (!ok) {
    resetDisplayHardware();
    display.begin();
    display.setPowerSave(0);
    display.setContrast(200);
    display.setBusClock(1000000UL);
  }
  showSplash("Display OK", "Avvio...", 400);

  dht.begin();

  // ADC config
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);

  randomSeed((uint32_t)esp_random());

  // Relè
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  setRelay(RELAY1, false);
  setRelay(RELAY2, false);
  setRelay(RELAY3, false);

  // Presenza
  pinMode(PRESENCE_PIN, INPUT);
  lastPresenceTime = millis();
  displayOn = true;
  display.setPowerSave(0);

  connectWiFi();

  wifiClient.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
    connectMQTTNonBloccante();
  }
  mqtt.loop();

  unsigned long now = millis();

  handlePresence();

  if (now - lastTelemetryMs >= telemetryIntMs) {
    lastTelemetryMs = now;
    updateSensors();
    publishTelemetry();
  }

  static unsigned long lastUiMs = 0;
  if (displayOn && (now - lastUiMs >= 80)) {
    lastUiMs = now;
    drawUI();
  }

  delay(1);
}



