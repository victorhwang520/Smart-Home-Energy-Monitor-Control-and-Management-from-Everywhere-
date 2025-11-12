
---

# esp32.ino

```cpp
/***************************************************
  Monitor de Energia IoT (ESP32)
  ZMPT101B (V) + SCT-013-000 (A) + MQTT + Blynk + API HTTP
  ---------------------------------------------------------
  - Ajuste credenciais Wi-Fi, MQTT e Blynk
  - Calibre CAL_ZMPT e CAL_SCT
****************************************************/

#if !defined(ESP32)
# error "Use ESP32. Para ESP8266, adote ADC externo (ADS1115) ou migre para ESP32."
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// ===== Blynk IoT =====
#define BLYNK_TEMPLATE_ID   "SEU_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "SEU_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "SEU_BLYNK_TOKEN"
#include <BlynkSimpleEsp32.h>

// ===== Wi-Fi / MQTT =====
const char* WIFI_SSID     = "SEU_WIFI";
const char* WIFI_PASS     = "SUA_SENHA";

const char* MQTT_BROKER   = "test.mosquitto.org";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENTID = "esp32_energy_monitor_01";

// Tópicos
const char* TOPIC_VRMS    = "iotbr/esp32/energy/vrms";
const char* TOPIC_IRMS    = "iotbr/esp32/energy/irms";
const char* TOPIC_PWR     = "iotbr/esp32/energy/power_apparent";
const char* TOPIC_JSON    = "iotbr/esp32/energy/json";

// ===== Pinos =====
// ADC1 somente (evita conflito com Wi-Fi)
const int PIN_ZMPT = 34;    // ZMPT101B
const int PIN_SCT  = 35;    // SCT-013-000
const int ADC_BITS = 12;
const float VREF   = 3.3f;

const int RELAY1_PIN = 25;
const int RELAY2_PIN = 26;

// ===== Blynk Vpins =====
#define VPIN_VRMS   V0
#define VPIN_IRMS   V1
#define VPIN_PWR    V2
#define VPIN_RLY1   V3
#define VPIN_RLY2   V4

// ===== Calibração =====
float CAL_ZMPT = 385.0f;  // V por "V RMS no ADC"
float CAL_SCT  = 60.0f;   // A por "V RMS no ADC"
float offsetZ  = 2048.0f; // iniciam em mid-ADC; auto-ajuste no setup
float offsetI  = 2048.0f;

const uint16_t SAMPLE_MS = 1000; // janela ~1s
const float ALPHA = 0.995f;      // filtro IIR p/ offset

// ===== Conectividade =====
WiFiClient espClient;
PubSubClient mqtt(espClient);
WebServer server(80);

// ===== Estados (expostos na API) =====
volatile float g_vrms = 0.0f, g_irms = 0.0f, g_S_va = 0.0f;
volatile bool g_r1 = false, g_r2 = false;

// ===== Util =====
static inline int readADC_Raw(int pin) { return analogRead(pin); }

void setRelay(uint8_t idx, bool on) {
  int pin = (idx == 1) ? RELAY1_PIN : RELAY2_PIN;
  digitalWrite(pin, on ? HIGH : LOW);   // ajuste se seu relé for active-LOW
  if (idx == 1) g_r1 = on; else g_r2 = on;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) delay(300);
}

void connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  uint8_t retry = 0;
  while (!mqtt.connected() && retry < 10) {
    if (mqtt.connect(MQTT_CLIENTID)) break;
    delay(500);
    retry++;
  }
}

// ===== Medição =====
void computeRMS(float &vrms, float &irms) {
  uint32_t t_start = millis();
  double accV2 = 0.0, accI2 = 0.0;
  uint32_t n = 0;

  while (millis() - t_start < SAMPLE_MS) {
    int rv = readADC_Raw(PIN_ZMPT);
    int ri = readADC_Raw(PIN_SCT);

    offsetZ = ALPHA * offsetZ + (1.0f - ALPHA) * rv;
    offsetI = ALPHA * offsetI + (1.0f - ALPHA) * ri;

    float vz = (float)rv - offsetZ;
    float iz = (float)ri - offsetI;

    float v_adc = vz * (VREF / ((1 << ADC_BITS) - 1));
    float i_adc = iz * (VREF / ((1 << ADC_BITS) - 1));

    accV2 += (double)v_adc * (double)v_adc;
    accI2 += (double)i_adc * (double)i_adc;
    n++;
    delayMicroseconds(200);
  }

  if (n == 0) { vrms = irms = 0; return; }
  float v_adc_rms = sqrt(accV2 / (double)n);
  float i_adc_rms = sqrt(accI2 / (double)n);

  vrms = v_adc_rms * CAL_ZMPT;
  irms = i_adc_rms * CAL_SCT;
}

// ===== Blynk Handlers =====
BLYNK_WRITE(VPIN_RLY1) { setRelay(1, param.asInt() == 1); }
BLYNK_WRITE(VPIN_RLY2) { setRelay(2, param.asInt() == 1); }

// ===== HTTP API =====
void handleMetrics() {
  char j[200];
  snprintf(j, sizeof(j),
    "{\"vrms\":%.2f,\"irms\":%.3f,\"S_va\":%.1f,\"r1\":%d,\"r2\":%d}",
    g_vrms, g_irms, g_S_va, g_r1 ? 1 : 0, g_r2 ? 1 : 0);
  server.send(200, "application/json", j);
}

void handleRelay() {
  if (!server.hasArg("ch") || !server.hasArg("on")) {
    server.send(400, "application/json", "{\"error\":\"args: ch,on\"}");
    return;
  }
  int ch = server.arg("ch").toInt();
  int on = server.arg("on").toInt();
  if (ch < 1 || ch > 2 || (on != 0 && on != 1)) {
    server.send(400, "application/json", "{\"error\":\"invalid ch/on\"}");
    return;
  }
  setRelay((uint8_t)ch, on == 1);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRoot() {
  server.send(200, "text/plain",
    "OK. Use /api/metrics (JSON) e /api/relay?ch=1&on=1. Abra o index.html no PC e aponte para o IP deste ESP.");
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(ADC_BITS);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  setRelay(1, false);
  setRelay(2, false);

  connectWiFi();
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(10000);

  connectMQTT();

  // offsets iniciais
  double avgZ = 0, avgI = 0;
  const int warm = 2000;
  for (int i = 0; i < warm; i++) { avgZ += readADC_Raw(PIN_ZMPT); avgI += readADC_Raw(PIN_SCT); delay(1); }
  offsetZ = (float)(avgZ / warm);
  offsetI = (float)(avgI / warm);

  // HTTP
  server.on("/", handleRoot);
  server.on("/api/metrics", HTTP_GET, handleMetrics);
  server.on("/api/relay",   HTTP_GET, handleRelay);
  server.begin();

  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

// ===== Loop =====
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();
  Blynk.run();
  server.handleClient();

  float vrms = 0.0f, irms = 0.0f;
  computeRMS(vrms, irms);
  float S_va = vrms * irms;

  g_vrms = vrms; g_irms = irms; g_S_va = S_va;

  // Serial
  Serial.printf("Vrms=%.1f V | Irms=%.2f A | S=%.1f VA\n", vrms, irms, S_va);

  // MQTT
  char buf[64];
  dtostrf(vrms, 0, 2, buf); mqtt.publish(TOPIC_VRMS, buf, true);
  dtostrf(irms, 0, 3, buf); mqtt.publish(TOPIC_IRMS, buf, true);
  dtostrf(S_va, 0, 1, buf); mqtt.publish(TOPIC_PWR,  buf, true);

  char j[160];
  snprintf(j, sizeof(j), "{\"vrms\":%.2f,\"irms\":%.3f,\"S_va\":%.1f}", vrms, irms, S_va);
  mqtt.publish(TOPIC_JSON, j, true);

  // Blynk
  Blynk.virtualWrite(VPIN_VRMS, vrms);
  Blynk.virtualWrite(VPIN_IRMS, irms);
  Blynk.virtualWrite(VPIN_PWR,  S_va);

  delay(1000);
}

/* ===== Notas =====
- Se seu módulo de relé for ativo-LOW, troque a lógica em setRelay().
- Para potência real (W) e PF, amostre v(t)*i(t) e integre num ciclo; se quiser te mando a versão.
*/
