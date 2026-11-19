/*
 * ============================================================================
 * UFES - LAB-AIR | Firmware Ponte Wi-Fi UDP <-> UART para Módulo ESP-07 (ESP8266)
 * Conecta o Quadrimotor (Teensy 4.1) ao Nó ROS no Computador do Laboratório
 * ============================================================================
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// ============================================================================
// 1. CONFIGURAÇÕES DE REDE WI-FI E PORTAS UDP
// ============================================================================
// Se USE_SOFT_AP = false, o ESP-07 conecta no roteador do LAB-AIR (Modo Station).
// Se USE_SOFT_AP = true, o ESP-07 cria sua própria rede Wi-Fi dedicada.
const bool USE_SOFT_AP = false;

const char* WIFI_SSID = "LABAIR_WIFI";     // Nome da rede Wi-Fi do laboratório
const char* WIFI_PASS = "senha_do_lab";    // Senha do Wi-Fi

// Configuração de IP Fixo (Recomendado para não mudar de IP a cada bateria)
const bool USE_STATIC_IP = true;
IPAddress local_IP(192, 168, 1, 150);      // IP fixo do Drone na rede do lab
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const unsigned int UDP_LOCAL_PORT = 8888;  // Porta onde o ESP-07 escuta comandos do ROS
const unsigned int UDP_REMOTE_PORT = 8889; // Porta para onde o ESP-07 envia telemetria ao PC

// ============================================================================
// 2. CONFIGURAÇÕES DE UART E WATCHDOG DE SEGURANÇA
// ============================================================================
const long UART_BAUD = 230400;             // Deve ser idêntico ao ESP_SERIAL na Teensy 4.1
const unsigned long WIFI_TIMEOUT_MS = 500; // Failsafe se ficar >500ms sem pacotes do ROS

WiFiUDP udp;
IPAddress rosComputerIP;
bool hasRosClient = false;
unsigned long lastUdpPacketTime = 0;
bool failsafeTriggered = true;

// Buffers não-bloqueantes
const int BUF_SIZE = 256;
char udpRxBuffer[BUF_SIZE];
char uartRxBuffer[BUF_SIZE];
int uartRxIndex = 0;

void setup() {
  // Inicializa UART conectada à Teensy 4.1 (Pinos TX/GPIO1 e RX/GPIO3 do ESP-07)
  Serial.begin(UART_BAUD);
  delay(100);

  // CRÍTICO: Desativa economia de energia do rádio para garantir baixa latência (< 5ms)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  if (USE_SOFT_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("UFES_DRONE_AP", "labair123", 6, 0, 2);
  } else {
    WiFi.mode(WIFI_STA);
    if (USE_STATIC_IP) {
      WiFi.config(local_IP, gateway, subnet);
    }
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(250);
    }
  }

  // Inicia socket UDP
  udp.begin(UDP_LOCAL_PORT);
  
  // Garante que a Teensy inicie desarmada
  Serial.println("DISARM");
}

void loop() {
  unsigned long now = millis();

  // --------------------------------------------------------------------------
  // 1. RECEPÇÃO WI-FI UDP (Nó ROS -> ESP-07 -> Teensy 4.1)
  // --------------------------------------------------------------------------
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    rosComputerIP = udp.remoteIP();
    hasRosClient = true;
    lastUdpPacketTime = now;
    failsafeTriggered = false;

    int len = udp.read(udpRxBuffer, BUF_SIZE - 1);
    if (len > 0) {
      udpRxBuffer[len] = '\0';
      // Repassa o comando imediatamente para a Teensy 4.1 via UART
      Serial.print(udpRxBuffer);
      if (udpRxBuffer[len - 1] != '\n') {
        Serial.print('\n');
      }
    }
  }

  // --------------------------------------------------------------------------
  // 2. RECEPÇÃO UART (Teensy 4.1 -> ESP-07 -> Nó ROS via UDP)
  // --------------------------------------------------------------------------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      uartRxBuffer[uartRxIndex] = '\0';
      if (uartRxIndex > 0 && hasRosClient) {
        udp.beginPacket(rosComputerIP, UDP_REMOTE_PORT);
        udp.write((const uint8_t*)uartRxBuffer, uartRxIndex);
        udp.endPacket();
      }
      uartRxIndex = 0;
    } else {
      if (uartRxIndex < BUF_SIZE - 1) {
        uartRxBuffer[uartRxIndex++] = c;
      } else {
        uartRxIndex = 0; // Proteção contra overflow de buffer
      }
    }
  }

  // --------------------------------------------------------------------------
  // 3. WATCHDOG DE COMUNICAÇÃO WI-FI (FAILSAFE)
  // --------------------------------------------------------------------------
  if (hasRosClient && !failsafeTriggered && (now - lastUdpPacketTime > WIFI_TIMEOUT_MS)) {
    // Se o PC/ROS parar de responder por mais de 500ms, desarma os motores na Teensy
    Serial.println("DISARM");
    failsafeTriggered = true;
  }
}