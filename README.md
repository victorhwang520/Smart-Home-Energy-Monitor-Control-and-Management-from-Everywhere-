# Detector de Gás (Gás de Cozinha / CH₄-like) com ESP32

> Projeto acadêmico IBM3119 • Protótipo open-source para detecção de gás inflamável usando **ESP32**, **sensor MQ-5** e **DHT11**, com envio via **MQTT**.

**Status atual**

* Hardware consolidado em **uma única placa**: **ESP32 WROVER** (dev kit Freenove)
  → não é mais necessário ESP8266 + Arduino Mega.
* Leitura do MQ-5 via **saída analógica (AOUT)** + conversão simples **ADC → ppm (regra de 3)**.
* Envio contínuo de dados para **broker Mosquitto** e visualização em **dashboard web**.
* Projeto pensado como **protótipo acadêmico ligado à tomada** (consumo alto para longas autonomias em bateria).

---

## 1) Requisitos do Projeto

### 1.1 Requisitos funcionais

* Detectar **aumento de concentração de gás inflamável** no ambiente (gás de cozinha / CH₄ equivalente).
* Medir e enviar:

  * **valor bruto do ADC** do MQ-5 (`raw_adc`);
  * **ppm estimado** (`ppm`) via modelo linear (200–10.000 ppm);
  * **temperatura e umidade** (DHT11).
* Publicar leituras via **Wi-Fi + MQTT** (Mosquitto).
* Permitir monitoramento em **frontend web** (HTML + JavaScript + MQTT/WebSocket).
* Possibilidade futura de **alarmar** (LEDs / buzzer).

### 1.2 Requisitos não funcionais

* Baixo custo e reprodutível com componentes comuns (ESP32 + MQ-5 + DHT11).
* Código aberto e comentado.
* **Aviso de segurança:** protótipo acadêmico, **não** substitui detector de gás certificado.
* Aceitação explícita de que o consumo é alto (MQ-5 + Wi-Fi) → foco em **fonte 5 V** ou bateria de grande capacidade por poucas horas.

---

## 2) Especificações do Sistema

### 2.1 Hardware

* **Microcontrolador:**

  * ESP32 WROVER (dev kit Freenove, 5 V em GPIO, ADC 12 bits).
* **Sensor de gás:**

  * **MQ-5** em módulo (heater + RL + comparador), alimentado em **5 V**.
  * Usando **apenas saída analógica (AOUT)** para estimar concentração.
* **Sensor de temperatura/umidade:**

  * **DHT11**, alimentado em 3,3 V.
* **Alimentação:**

  * Fonte USB 5 V ou power bank (ex.: 10.000 mAh → ~1 dia de operação contínua).
* **Protoboard/jumpers** para a montagem.

### 2.2 Software (firmware ESP32)

* Ambiente:

  * **Arduino IDE** com core **“esp32 by Espressif Systems”**.
* Bibliotecas usadas:

  * `WiFi.h` (nativo do core ESP32)
  * `PubSubClient` (MQTT)
  * `DHT` + `Adafruit Unified Sensor` (para DHT11)
* Funcionalidades:

  * Leitura analógica do MQ-5 (GPIO34) com média de N amostras.
  * Conversão **ADC bruto → ppm estimado** via **regra de 3 linear**:

    * abaixo de `ADC = 250` → `ppm = 0` (ruído / gás irrisório);
    * `ADC = 250` → ~**200 ppm**;
    * `ADC = 4095` → ~**10.000 ppm**;
    * interpolação linear entre esses pontos.
  * Leitura de **temperatura** e **umidade** pelo DHT11.
  * Publicação periódica (a cada 5 s) em:

    * `gas/dht` → `{"temp":..., "hum":...}`
    * `gas/mq`  → `{"raw_adc":..., "v_adc":..., "ppm":..., "temp":..., "hum":...}`

> ⚠️ A escala de ppm é **didática / aproximada**, baseada em faixa típica (200–10.000 ppm) do MQ-5 e no range do ADC do ESP32. Não é uma calibração metrológica certificada.

---

## 3) Conexões de Hardware

### 3.1 Ligações principais (ESP32 WROVER)

| Função       | ESP32 (GPIO) | Observações                                         |
| ------------ | ------------ | --------------------------------------------------- |
| MQ-5 – VCC   | 5V           | Heater consome ~160 mA (@5 V)                       |
| MQ-5 – GND   | GND          | Referência comum                                    |
| MQ-5 – AOUT  | GPIO34       |                                                     |
| DHT11 – DATA | GPIO4        | DHT11 alimentado em 3,3 V                           |
| DHT11 – VCC  | 3V3          |                                                     |
| DHT11 – GND  | GND          |                                                     |


---

## 4) Protocolo de Medição e Modelo de ppm

### 4.1 Leitura do MQ-5 (bruto)

1. O ESP32 lê o AOUT do MQ-5 no **GPIO34** via `analogRead()` (12 bits → 0..4095).
2. É feita a média de várias leituras (ex.: 10 amostras) para reduzir ruído.
3. O valor médio é o **`raw_adc`**.

A tensão aproximada no pino é calculada como:

[
V_{ADC} = \text{raw_adc} \times \frac{3{,}3\text{ V}}{4095}
]

Esse valor é enviado como `v_adc` no JSON.

### 4.2 Conversão ADC → ppm (regra de 3)

Para simplificar o uso acadêmico, foi adotado:

* Faixa útil do MQ-5 (datasheet): **200 a 10.000 ppm** (gás combustível).
* Mapeamento escolhido:

  * `raw_adc < 250` → `ppm = 0`
  * `raw_adc = 250` → `ppm = 200`
  * `raw_adc = 4095` → `ppm = 10.000`

Para `raw_adc` entre 250 e 4095:

[
\text{frac} = \frac{\text{raw_adc} - 250}{4095 - 250}
]

[
\text{ppm} = 200 + \text{frac} \cdot (10000 - 200)
]

No código:

```cpp
float adcToPPM(int adc) {
  if (adc < 250) return 0.0;

  if (adc > 4095) adc = 4095;

  float frac = (float)(adc - 250) / (float)(4095 - 250);
  float ppm  = 200.0 + frac * (10000.0 - 200.0);
  return ppm;
}
```

> De novo: isso é um **modelo linear simplificado**, não a curva logarítmica oficial do datasheet. Foi escolhido para facilitar visualização e explicação em sala, mantendo monotonicidade (mais gás → ADC maior → ppm maior).

### 4.3 Cuidados com ventilação e jatos de gás (Princípio de Bernoulli)

Os melhores resultados são obtidos quando o sensor está em um ambiente com ar relativamente **estacionário**, sem jatos fortes de gás ou vento direto (ventilador, exaustor, janela aberta com corrente forte).

Quando se joga um jato de gás diretamente sobre o sensor, ou se há um fluxo de ar muito rápido ao redor, entram em jogo efeitos de **dinâmica dos fluidos**, incluindo o Princípio de Bernoulli:

- regiões com **gás em alta velocidade** podem gerar **zonas de baixa pressão** e puxar ar atmosférico junto;
- o jato cria uma **mistura turbulenta** (gás + ar “limpo”) que varia muito no tempo;
- o fluxo pode inclusive **afastar** o gás da região do sensor depois do pico inicial.

Na prática, isso causa comportamentos como:
- pico rápido de leitura quando o jato atinge o sensor;
- queda parcial enquanto o jato continua (“ar fresco” sendo arrastado);
- aumento novamente quando o jato é desligado e o gás fica mais “estacionário” no ambiente.

Por isso, recomenda-se:
- **não instalar o sensor em locais com vento forte constante** (exaustores, dutos de ventilação, correntes de ar intensas);
- **evitar testar apontando jatos muito fortes diretamente no sensor**;
- preferir testes em que o gás se mistura no ar de forma mais estável (ex.: pequeno volume de gás liberado em um recipiente ou ambiente controlado).


---

## 5) Comunicação MQTT

* **Broker:** Mosquitto rodando em rede local.
* **Conexão:** TCP/IP (porta 1883, sem TLS) usando `PubSubClient`.
* **Tópicos:**

### 5.1 `gas/dht`

Payload (JSON):

```json
{
  "temp": 26.5,
  "hum":  55.2
}
```

### 5.2 `gas/mq`

Payload (JSON):

```json
{
  "raw_adc": 720,
  "v_adc":   0.580,
  "ppm":     3200.5,
  "temp":    26.5,
  "hum":     55.2
}
```

Esses dados são consumidos por um **frontend web** em HTML/JavaScript que se conecta ao broker (via WebSocket) e exibe:

* histórico de temperatura/umidade,
* valor bruto do ADC,
* ppm estimado,
* alertas quando ppm passar de um limiar e/ou temperatura ficar muito alta.

---

## 6) Consumo de Energia e Autonomia

### 6.1 Estimativa de corrente

Componentes principais:

* **MQ-5** (heater a 5 V): ~**160 mA**
* **ESP32 WROVER** com Wi-Fi ativo: ~**120 mA** (típico)
* **DHT11:** ~**1 mA** (pico durante leitura)

Total:

> **Corrente total ≈ 280–300 mA**

### 6.2 Bateria 10.000 mAh

Para uma bateria de **10.000 mAh**, com corrente média de ~300 mA:

[
t_{\text{teórico}} \approx \frac{10000}{300} \approx 33\ \text{horas}
]

Considerando perdas (eficiência do conversor, bateria não entrega 100% etc.), o tempo real é mais conservador:

> **Autonomia estimada: ~26–30 horas de operação contínua.**

Conclusão:

* Este arranjo (MQ-5 + ESP32 + Wi-Fi) é mais adequado a:

  * uso **ligado na tomada / fonte 5 V**,
  * ou uso em bateria por **períodos curtos** (horas ou 1 dia),
  * do que para semanas/meses de operação contínua.

---

## 7) Limitações e Trabalhos Futuros

* **Limitações:**

  * MQ-5 consome muita energia (heater em 5 V).
  * Modelo de ppm é **linear e didático**, não substitui calibração em câmara de gás.
  * Autonomia em bateria é limitada (~1 dia, mesmo com 10.000 mAh).

* **Possíveis melhorias:**

  * Implementar **duty-cycle** do heater (ligar/medir/desligar) para reduzir consumo.
  * Usar **deep sleep** no ESP32 entre medições (ex.: 1 leitura a cada 1 min).
  * Adotar sensores de gás **eletroquímicos** de baixo consumo para aplicações “a pilha”.
  * Incluir **alarmes visuais/sonoros** (LEDs, buzzer) e thresholds configuráveis.
  * Melhorar o modelo de ppm usando curva logarítmica ajustada a partir de testes controlados.

---

## 8) Estrutura sugerida do repositório

```text
/docs/           (datasheets, notas de consumo, modelos de ppm)
/hardware/       (esquemas de ligação, fotos da montagem)
/firmware/       (código do ESP32: .ino, configs MQTT/Wi-Fi)
/frontend/       (HTML + JS para dashboard MQTT)
/test/           (logs de testes, capturas do broker e do serial)
/LICENSE
README.md
```

---
