#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ========== CONFIG Wi-Fi ==========
const char* WIFI_SSID = "Nome_wifi";
const char* WIFI_PASS = "Senha_wifi";

// ========== CONFIG MQTT ==========
const char* MQTT_SERVER = "Ip_do_server";
const int   MQTT_PORT   = 1883;
const char* TOPIC_DHT   = "gas/dht";
const char* TOPIC_MQ    = "gas/mq";

WiFiClient espClient;
PubSubClient client(espClient);

// ========== PINOS ==========
#define PIN_MQ   34    // AOUT do MQ-5 (via divisor de tensão)
#define PIN_DHT  4     // DATA do DHT11

// ========== DHT11 ==========
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

// ========== ADC / MQ-5 ==========
const float ADC_VREF = 3.3;   // ESP32 ~3.3 V de referencia

// Mapeamento linear ADC -> ppm (regra de 3)
//  - ADC_MIN  (250)  ->  200 ppm
//  - ADC_MAX  (4095) ->  10000 ppm
//  - abaixo de 250   ->  0 ppm (desprezível)
const int   ADC_MIN  = 250;       // ponto de 200 ppm
const int   ADC_MAX  = 4095;      // ponto de 10000 ppm
const float PPM_MIN  = 200.0;
const float PPM_MAX  = 10000.0;

// Converte ADC bruto para ppm pela regra de 3
float adcToPPM(int adc) {
  // abaixo do limiar, consideramos 0 ppm
  if (adc < ADC_MIN) {
    return 0.0;
  }

  // satura em ADC_MAX pra não extrapolar demais
  if (adc > ADC_MAX) {
    adc = ADC_MAX;
  }

  float frac = (float)(adc - ADC_MIN) / (float)(ADC_MAX - ADC_MIN);
  float ppm  = PPM_MIN + frac * (PPM_MAX - PPM_MIN);
  return ppm;
}

// ========== Wi-Fi / MQTT ==========
void connectWiFi() {
  Serial.print("Conectando ao Wi-Fi ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi OK, IP = ");
  Serial.println(WiFi.localIP());
}

void mqttCallback(char*, byte*, unsigned int) {
  // sem subscribe
}

void connectMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);

  while (!client.connected()) {
    Serial.print("Conectando ao MQTT... ");
    if (client.connect("ESP32_GAS_NODE")) {
      Serial.println("OK");
    } else {
      Serial.print("falha (rc=");
      Serial.print(client.state());
      Serial.println(") - tentando em 5 s");
      delay(5000);
    }
  }
}

// ========== TEMPORIZAÇÃO ==========
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 WROVER - MQ-5 (analogico bruto) + DHT11 + MQTT");
  Serial.print("Mapeamento linear: ADC ");
  Serial.print(ADC_MIN);
  Serial.print(" -> ");
  Serial.print(PPM_MIN);
  Serial.print(" ppm | ADC ");
  Serial.print(ADC_MAX);
  Serial.print(" -> ");
  Serial.print(PPM_MAX);
  Serial.println(" ppm (abaixo de 250 = 0 ppm)");

  pinMode(PIN_MQ, INPUT);
  analogReadResolution(12);   // 0..4095

  dht.begin();

  connectWiFi();
  connectMQTT();
}

// ========== LOOP ==========
void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // --- MQ-5: valor bruto do analogRead ---
  const int samples = 10;
  long adcSum = 0;
  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(PIN_MQ);
    delay(5);
  }
  int raw_adc = (int)(adcSum / samples);         // valor bruto
  float v_adc = raw_adc * (ADC_VREF / 4095.0);   // tensão aproximada no pino
  float ppm   = adcToPPM(raw_adc);               // ppm pela regra de 3

  // --- DHT11 ---
  float hum  = dht.readHumidity();
  float temp = dht.readTemperature();

  // --- SERIAL BONITINHO ---
  Serial.print("[MQ-5] raw_adc=");
  Serial.print(raw_adc);
  Serial.print(" | v_adc=");
  Serial.print(v_adc, 3);
  Serial.print(" V | ppm≈");
  Serial.print(ppm, 1);

  Serial.print("   ||   [DHT11] ");
  if (isnan(temp) || isnan(hum)) {
    Serial.print("temp=ERR, hum=ERR");
  } else {
    Serial.print("temp=");
    Serial.print(temp, 1);
    Serial.print(" °C, hum=");
    Serial.print(hum, 1);
    Serial.print(" %");
  }
  Serial.println();

  // --- MQTT (a cada 5 s) ---
  unsigned long now = millis();
  if (now - lastPublish > PUBLISH_INTERVAL) {
    lastPublish = now;

    // DHT
    if (!isnan(temp) && !isnan(hum)) {
      char payloadDHT[64];
      snprintf(payloadDHT, sizeof(payloadDHT),
               "{\"temp\":%.1f,\"hum\":%.1f}",
               temp, hum);
      client.publish(TOPIC_DHT, payloadDHT);
      Serial.print("Publicado DHT -> ");
      Serial.println(payloadDHT);
    } else {
      Serial.println("Falha ao ler DHT11 (nao publicado)");
    }

    // MQ-5
    char payloadMQ[160];
    snprintf(payloadMQ, sizeof(payloadMQ),
             "{\"raw_adc\":%d,"
             "\"v_adc\":%.3f,"
             "\"ppm\":%.1f,"
             "\"temp\":%.1f,"
             "\"hum\":%.1f}",
             raw_adc,
             v_adc,
             ppm,
             isnan(temp) ? 0.0 : temp,
             isnan(hum)  ? 0.0 : hum);

    client.publish(TOPIC_MQ, payloadMQ);
    Serial.print("Publicado MQ -> ");
    Serial.println(payloadMQ);
  }

  delay(200);
}
