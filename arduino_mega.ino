/***************************************************
  ARDUINO MEGA 2560 + Ethernet (W5100/W5500)
  ZMPT101B (tensão) em A0
  SCT-013-000 (corrente) em A1
  2 relés em D22 e D23
  MQTT (PubSubClient) + Blynk (Ethernet)

  Bibliotecas:
  - Ethernet (padrão Arduino, W5100/W5500)
  - PubSubClient
  - Blynk (BlynkSimpleEthernet.h)

  Ajuste:
  - BLYNK_AUTH_TOKEN
  - MQTT_BROKER
  - CAL_ZMPT / CAL_SCT (calibração)
****************************************************/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// ===== Blynk (Ethernet) =====
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID   "SEU_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "SEU_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "SEU_BLYNK_TOKEN"
#include <BlynkSimpleEthernet.h>

// ===== Pinos =====
const uint8_t PIN_ZMPT = A0;   // tensão
const uint8_t PIN_SCT  = A1;   // corrente
const uint8_t RELAY1_PIN = 22;
const uint8_t RELAY2_PIN = 23;

// ===== Blynk Vpins =====
#define VPIN_VRMS   V0
#define VPIN_IRMS   V1
#define VPIN_PWR    V2
#define VPIN_RLY1   V3
#define VPIN_RLY2   V4

// ===== MQTT =====
const char* MQTT_BROKER   = "test.mosquitto.org";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENTID = "mega_energy_monitor_01";
const char* TOPIC_VRMS    = "iotbr/mega/energy/vrms";
const char* TOPIC_IRMS    = "iotbr/mega/energy/irms";
const char* TOPIC_PWR     = "iotbr/mega/energy/power_apparent";
const char* TOPIC_JSON    = "iotbr/mega/energy/json";

// ===== Ethernet =====
// Se quiser MAC fixo, defina um:
byte mac[] = { 0xDE,0xAD,0xBE,0xEF,0x12,0x34 };
// DHCP por padrão

EthernetClient ethClient;
PubSubClient mqtt(ethClient);

// ===== ADC / Calibração =====
// Mega: ADC 10 bits (0..1023), VREF padrão 5.0V (confira o seu!)
const uint16_t ADC_MAX = 1023;
const float    VREF    = 5.0f;

// Converte Vrms_adc -> Volts RMS reais (ajuste com multímetro)
float CAL_ZMPT = 130.0f;  // ajuste na prática (depende do trimpot do ZMPT)
// Converte Irms_adc -> Amperes RMS (ajuste com carga conhecida)
float CAL_SCT  = 26.0f;   // ajuste típico; refine testando

// Offset DC (auto-ajuste)
float offsetZ = ADC_MAX/2.0f;
float offsetI = ADC_MAX/2.0f;

// Janela e filtro
const uint16_t SAMPLE_MS = 1000;   // ~1s
const float ALPHA = 0.995f;

// ===== Estados =====
volatile bool g_r1=false, g_r2=false;

// ===== Funções =====
void setRelay(uint8_t idx, bool on) {
  uint8_t pin = (idx==1) ? RELAY1_PIN : RELAY2_PIN;
  digitalWrite(pin, on ? HIGH : LOW);   // ajuste se seu relé for ativo-LOW
  if (idx==1) g_r1=on; else g_r2=on;
}

BLYNK_WRITE(VPIN_RLY1){ setRelay(1, param.asInt()==1); }
BLYNK_WRITE(VPIN_RLY2){ setRelay(2, param.asInt()==1); }

void connectEthernet() {
  if (Ethernet.linkStatus() == LinkOFF) {
    // tenta reinicializar (alguns shields precisam de reset)
  }
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("Sem Ethernet shield!"));
  }
  static bool started=false;
  if(!started){
    if (Ethernet.begin(mac)==0){ // DHCP
      Serial.println(F("DHCP falhou, tentando IP fixo 192.168.0.177"));
      IPAddress ip(192,168,0,177), dns(8,8,8,8), gw(192,168,0,1), mask(255,255,255,0);
      Ethernet.begin(mac, ip, dns, gw, mask);
    }
    started=true;
    delay(1000);
    Serial.print(F("IP: ")); Serial.println(Ethernet.localIP());
  }
}

void connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  uint8_t retry=0;
  while (!mqtt.connected() && retry<10) {
    if (mqtt.connect(MQTT_CLIENTID)) break;
    delay(500);
    retry++;
  }
}

static inline uint16_t adcRead(uint8_t pin){ return analogRead(pin); }

void computeRMS(float &vrms, float &irms) {
  uint32_t t0 = millis();
  double accV2=0, accI2=0; uint32_t n=0;
  while (millis()-t0 < SAMPLE_MS) {
    uint16_t rv = adcRead(PIN_ZMPT);
    uint16_t ri = adcRead(PIN_SCT);

    offsetZ = ALPHA*offsetZ + (1.0f-ALPHA)*rv;
    offsetI = ALPHA*offsetI + (1.0f-ALPHA)*ri;

    float vz = (float)rv - offsetZ;
    float iz = (float)ri - offsetI;

    // converte contagens -> volts no ADC
    float v_adc = vz * (VREF / ADC_MAX);
    float i_adc = iz * (VREF / ADC_MAX);

    accV2 += (double)v_adc * (double)v_adc;
    accI2 += (double)i_adc * (double)i_adc;
    n++;
    delayMicroseconds(200);
  }
  if (!n){ vrms=irms=0; return; }

  float v_adc_rms = sqrt(accV2 / (double)n);
  float i_adc_rms = sqrt(accI2 / (double)n);

  vrms = v_adc_rms * CAL_ZMPT; // Volts reais
  irms = i_adc_rms * CAL_SCT;  // Amperes reais
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

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  setRelay(1,false); setRelay(2,false);

  // Ethernet + Blynk
  connectEthernet();
  Blynk.begin(BLYNK_AUTH_TOKEN); // via Ethernet
  connectMQTT();

  // pré-offset
  double az=0, ai=0; for(int i=0;i<2000;i++){ az += adcRead(PIN_ZMPT); ai += adcRead(PIN_SCT); delay(1); }
  offsetZ = az/2000.0; offsetI = ai/2000.0;
}

void loop(){
  Blynk.run();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  float vrms=0, irms=0;
  computeRMS(vrms, irms);
  float S_va = vrms * irms;

  // Debug
  Serial.print(F("Vrms="));Serial.print(vrms,1);
  Serial.print(F(" V | Irms="));Serial.print(irms,2);
  Serial.print(F(" A | S="));Serial.print(S_va,1);Serial.println(F(" VA"));

  // Blynk KPIs
  Blynk.virtualWrite(VPIN_VRMS, vrms);
  Blynk.virtualWrite(VPIN_IRMS, irms);
  Blynk.virtualWrite(VPIN_PWR,  S_va);

  // MQTT
  publishMQTT(vrms, irms, S_va);

  delay(1000);
}

/* Notas:
 - Se relé for ativo-LOW, inverta a lógica em setRelay().
 - Para potência real (W) e PF, amostrar v(t)*i(t) e integrar num ciclo.
*/
