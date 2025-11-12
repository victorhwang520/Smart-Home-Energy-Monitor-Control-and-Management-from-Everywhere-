/***************************************************
  ESP8266 NodeMCU + ADS1115 (I2C) para 2 ADCs
  ZMPT101B (tensão) no ADS A0
  SCT-013-000 (corrente) no ADS A1
  2 relés em D5 e D6
  Wi-Fi + MQTT (PubSubClient) + Blynk (ESP8266)

  Bibliotecas:
  - ESP8266WiFi
  - PubSubClient
  - Blynk (BlynkSimpleEsp8266.h)
  - Adafruit_ADS1X15 (ADS1115)

  Ligações I2C ESP8266:
  - SDA = D2 (GPIO4)
  - SCL = D1 (GPIO5)

  Ajuste:
  - WIFI_SSID / WIFI_PASS
  - BLYNK_AUTH_TOKEN
  - MQTT_BROKER
  - GAIN do ADS (mV/LSB) e calibração CAL_ZMPT / CAL_SCT
****************************************************/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>

// ===== Blynk =====
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID   "SEU_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "SEU_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "SEU_BLYNK_TOKEN"
#include <BlynkSimpleEsp8266.h>

// ===== Credenciais =====
const char* WIFI_SSID = "SEU_WIFI";
const char* WIFI_PASS = "SUA_SENHA";

// ===== MQTT =====
const char* MQTT_BROKER   = "test.mosquitto.org";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENTID = "esp8266_energy_monitor_01";
const char* TOPIC_VRMS    = "iotbr/esp8266/energy/vrms";
const char* TOPIC_IRMS    = "iotbr/esp8266/energy/irms";
const char* TOPIC_PWR     = "iotbr/esp8266/energy/power_apparent";
const char* TOPIC_JSON    = "iotbr/esp8266/energy/json";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===== ADS1115 =====
Adafruit_ADS1115 ads;
// GAIN: define alcance e mV/LSB
// GAIN_TWOTHIRDS: 0.1875 mV/LSB (±6.144 V)
// Ajuste o GAIN conforme seu circuito de condicionamento (ZMPT/SCT + burden).
const adsGain_t ADS_GAIN = GAIN_TWOTHIRDS;
float LSB_mV = 0.1875f; // mV por LSB nesse ganho

// ===== Pinos Relé =====
const uint8_t RELAY1_PIN = D5;  // GPIO14
const uint8_t RELAY2_PIN = D6;  // GPIO12

// ===== Blynk Vpins =====
#define VPIN_VRMS   V0
#define VPIN_IRMS   V1
#define VPIN_PWR    V2
#define VPIN_RLY1   V3
#define VPIN_RLY2   V4

// ===== Calibração =====
// Convertem RMS no ADC (em Volts) para unidades reais
float CAL_ZMPT = 260.0f;  // ajuste prático (depende do trimpot do ZMPT e do ganho)
float CAL_SCT  = 50.0f;   // ajuste prático conforme burden e cabo

// Offset DC em LSB (ADS é signed 16-bit)
float offsetZ = 0.0f;
float offsetI = 0.0f;

const uint16_t SAMPLE_MS = 1000;
const float ALPHA = 0.995f;

// ===== Estados =====
volatile bool g_r1=false, g_r2=false;

// ===== Utils =====
void setRelay(uint8_t idx, bool on){
  uint8_t pin = (idx==1)? RELAY1_PIN : RELAY2_PIN;
  digitalWrite(pin, on ? HIGH : LOW); // ajuste se for ativo-LOW
  if (idx==1) g_r1=on; else g_r2=on;
}

BLYNK_WRITE(VPIN_RLY1){ setRelay(1, param.asInt()==1); }
BLYNK_WRITE(VPIN_RLY2){ setRelay(2, param.asInt()==1); }

void connectWiFi(){
  if (WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while (WiFi.status()!=WL_CONNECTED && (millis()-t0)<20000) delay(300);
}

void connectMQTT(){
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  uint8_t retry=0;
  while (!mqtt.connected() && retry<10){
    if (mqtt.connect(MQTT_CLIENTID)) break;
    delay(500);
    retry++;
  }
}

// Lê canal do ADS e devolve tensão no ADC (em Volts), removendo offset
float readADS_V(uint8_t channel, float &offset) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  // atualiza offset IIR no domínio "raw"
  offset = ALPHA*offset + (1.0f-ALPHA)*(float)raw;
  float centered = (float)raw - offset;        // LSB centrado
  float mV = centered * LSB_mV;               // mV no ADC
  return mV / 1000.0f;                        // Volts no ADC
}

void computeRMS(float &vrms, float &irms){
  uint32_t t0=millis();
  double accV2=0, accI2=0; uint32_t n=0;

  while (millis()-t0 < SAMPLE_MS){
    float v_adc = readADS_V(/*A0*/0, offsetZ); // ZMPT
    float i_adc = readADS_V(/*A1*/1, offsetI); // SCT

    accV2 += (double)v_adc * (double)v_adc;
    accI2 += (double)i_adc * (double)i_adc;
    n++;
    delayMicroseconds(300);
  }
  if (!n){ vrms=irms=0; return; }

  float v_adc_rms = sqrt(accV2/(double)n);
  float i_adc_rms = sqrt(accI2/(double)n);

  vrms = v_adc_rms * CAL_ZMPT; // V reais
  irms = i_adc_rms * CAL_SCT;  // A reais
}

void publishMQTT(float vrms, float irms, float Sva){
  char b[64], j[160];
  dtostrf(vrms,0,2,b); mqtt.publish(TOPIC_VRMS,b,true);
  dtostrf(irms,0,3,b); mqtt.publish(TOPIC_IRMS,b,true);
  dtostrf(Sva ,0,1,b); mqtt.publish(TOPIC_PWR ,b,true);
  snprintf(j,sizeof(j), "{\"vrms\":%.2f,\"irms\":%.3f,\"S_va\":%.1f,\"r1\":%d,\"r2\":%d}",
           vrms, irms, Sva, g_r1?1:0, g_r2?1:0);
  mqtt.publish(TOPIC_JSON, j, true);
}

void setupADS(){
  if(!ads.begin()){
    Serial.println(F("ADS1115 não encontrado!"));
    while(1) delay(10);
  }
  ads.setGain(ADS_GAIN);
  // LSB_mV conforme ganho
  switch(ADS_GAIN){
    case GAIN_TWOTHIRDS: LSB_mV=0.1875f; break; // ±6.144V
    case GAIN_ONE:       LSB_mV=0.1250f; break; // ±4.096V
    case GAIN_TWO:       LSB_mV=0.0625f; break; // ±2.048V
    case GAIN_FOUR:      LSB_mV=0.03125f; break;// ±1.024V
    case GAIN_EIGHT:     LSB_mV=0.015625f; break;// ±0.512V
    case GAIN_SIXTEEN:   LSB_mV=0.0078125f; break;// ±0.256V
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  setRelay(1,false); setRelay(2,false);

  setupADS();

  connectWiFi();
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  connectMQTT();

  // pré-offset (ADS)
  double az=0, ai=0;
  for(int i=0;i<2000;i++){
    az += ads.readADC_SingleEnded(0);
    ai += ads.readADC_SingleEnded(1);
    delay(1);
  }
  offsetZ = az/2000.0;
  offsetI = ai/2000.0;

  Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
}

void loop(){
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();
  Blynk.run();

  float vrms=0, irms=0;
  computeRMS(vrms, irms);
  float S_va = vrms * irms;

  // Debug
  Serial.printf("Vrms=%.1f V | Irms=%.2f A | S=%.1f VA\n", vrms, irms, S_va);

  // Blynk KPIs
  Blynk.virtualWrite(VPIN_VRMS, vrms);
  Blynk.virtualWrite(VPIN_IRMS, irms);
  Blynk.virtualWrite(VPIN_PWR,  S_va);

  // MQTT
  publishMQTT(vrms, irms, S_va);

  delay(1000);
}

/* Notas:
 - ADS1115 dá folga pra ler tensão e corrente ao mesmo tempo no ESP8266.
 - Ajuste GAIN conforme seu circuito (pra não saturar a entrada do ADS).
 - Se o relé for ativo-LOW, inverta a escrita em setRelay().
 - Pra potência real (W) e PF, posso te mandar a versão com v(t)*i(t).
*/
