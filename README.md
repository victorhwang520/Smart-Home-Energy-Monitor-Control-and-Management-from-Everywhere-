# Monitor de Energia IoT (ESP32) — ZMPT101B + SCT-013-000 + MQTT + Blynk

Sistema de monitoramento de **tensão (ZMPT101B)** e **corrente (SCT-013-000 100A)** com **ESP32**, publicação em **MQTT**, painel **Blynk IoT** e **API HTTP local** (com `index.html` simples) para visualização e controle de **2 relés**.

## Componentes
- **ESP32** (ou ESP8266 NodeMCU com ADC externo — ver nota).
- **ZMPT101B** (sensor de tensão AC 0–250 V — isolamento por trafo).
- **SCT-013-000 100 A** (sensor de corrente não invasivo — *use o módulo com resistor burden*).
- **2x Relés** (controle e proteção das cargas).
- **Blynk IoT** (telemetria e interface móvel).
- **MQTT** (broker local ou público).
- **ZigBee/PLC (opcional)**: integração futura com geração **solar/eólica**.

> **ESP8266**: possui apenas 1 ADC. Para medir tensão **e** corrente simultaneamente, use **ADS1115** (I²C) ou migre para **ESP32** (recomendado).

## Ligações (ESP32)
- **ZMPT101B → GPIO34 (ADC1_CH6)**  
- **SCT-013-000 → GPIO35 (ADC1_CH7)**  
- **Relé 1 → GPIO25**  
- **Relé 2 → GPIO26**  
- **Alimentação**: conforme sua placa/módulos. **Isolamento e segurança em AC são obrigatórios**.

> Muitos módulos de relé são **ativo-LOW**. O sketch já abstrai; ajuste se necessário.

## Bibliotecas (Arduino IDE)
- **Blynk** (IoT, `BlynkSimpleEsp32.h`)
- **PubSubClient** (MQTT)
- **WiFi.h** (nativo ESP32)

## Credenciais
No `esp32.ino`, ajuste:
```cpp
const char* WIFI_SSID   = "SEU_WIFI";
const char* WIFI_PASS   = "SUA_SENHA";
#define BLYNK_AUTH_TOKEN "SEU_BLYNK_TOKEN"
const char* MQTT_BROKER = "test.mosquitto.org"; // troque p/ seu broker
