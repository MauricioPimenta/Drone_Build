#include <Arduino.h>
#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// --- Definições de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int VOLTAGE_PIN = A0;  // Pino analógico 14 da Teensy para ler a tensão

// --- Configurações do Ensaio ---
const int SAMPLES_TO_AVERAGE = 10;        // Quantidade de amostras para cada média salva
const float CALIBRATION_FACTOR = 113.0f; // Substitua pelo seu fator calibrado

// --- Configurações do ADC (Leitura de Tensão) ---
const int ADC_RESOLUTION_BITS = 12;  // Resolução configurável (ex: 8, 10, 12, 14, 16...)

// Calcula automaticamente 2^Bits em tempo de compilação (ex: 1 << 12 = 4096)
const float ADC_MAX_VAL = (float)(1 << ADC_RESOLUTION_BITS);

const float VREF = 3.3f;                  // Tensão de referência interna da Teensy

/* 
 * FATOR DO DIVISOR DE TENSÃO
 * Se estiver usando um divisor de tensão com resistores R1 e R2, 
 * a fórmula é: VOLTAGE_DIVIDER_RATIO = (R1 + R2) / R2
 * Se a fonte for direto para o pino (máximo absoluto 3.3V!), deixe = 1.0
 */
const float VOLTAGE_DIVIDER_RATIO = 1.0f; 

HX711 scale;
File dataFile;
char logFileName[64]; 

// --- Variáveis de Acúmulo e Médias ---
int samplesCollected = 0;
float thrustSum = 0.0;
float adcSum = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("--- Setup de Inicialização (Datalogger) ---");

  // 1. Configuração do ADC
  analogReadResolution(ADC_RESOLUTION_BITS);
  // Opcional: analogReadAveraging(4); // Pode usar a média nativa do hardware se quiser

  // 2. Inicialização do SD Card e Indexação
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Falha no SD card! Verifique o hardware.");
    while (1);
  }
  
  int fileIndex = 1;
  while (true) {
    sprintf(logFileName, "curva_tensao_%03d.csv", fileIndex);
    if (!SD.exists(logFileName)) {
      break; 
    }
    fileIndex++;
    if (fileIndex > 999) {
      Serial.println("Limite de arquivos atingido!");
      while (1);
    }
  }

  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage_V,Thrust_unit"); // Novo cabeçalho!
    dataFile.close();
    Serial.print("Novo arquivo de log criado: ");
    Serial.println(logFileName);
  } else {
    Serial.println("Erro ao criar o arquivo no SD.");
  }

  // 3. Inicialização da Célula de Carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  
  Serial.println("Tarando a balança... Remova qualquer peso do motor.");
  scale.tare(20); 
  Serial.println("Balança zerada.");
  
  Serial.println("=== SISTEMA PRONTO ===");
  Serial.println("Você já pode começar a variar a tensão no ESC.");
  Serial.println("Salvando dados continuamente...");
}

void loop() {
  // O código usa o HX711 como "clock" da amostragem (10Hz ou 80Hz)
  if (scale.is_ready()) {
    
    // 1. Lê os dados brutos no mesmo instante de tempo
    float currentThrust = scale.get_units(1);
    float currentAdc = analogRead(VOLTAGE_PIN);

    // 2. Acumula os valores
    thrustSum += currentThrust;
    adcSum += currentAdc;
    samplesCollected++;

    // 3. Quando atingir a janela de amostras desejada, calcula e salva
    if (samplesCollected >= SAMPLES_TO_AVERAGE) {
      
      // Médias da janela
      float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
      float avgAdc = adcSum / SAMPLES_TO_AVERAGE;
      
      // Conversão matemática do ADC para Volts reais
      // (Leitura Média / 4095) * 3.3V * Razão do Divisor
      float actualVoltage = (avgAdc / ADC_MAX_VAL) * VREF * VOLTAGE_DIVIDER_RATIO;
      
      // Log no Monitor Serial (Feedback visual na bancada)
      Serial.print("Tensão Aplicada: ");
      Serial.print(actualVoltage, 3);
      Serial.print(" V \t| Empuxo: ");
      Serial.println(avgThrust, 3);
      
      // Log no Cartão SD (Abre, escreve, fecha para garantir a integridade dos dados)
      dataFile = SD.open(logFileName, FILE_WRITE);
      if (dataFile) {
        dataFile.print(actualVoltage, 4); // 4 casas decimais para tensão no arquivo
        dataFile.print(",");
        dataFile.println(avgThrust, 3); 
        dataFile.close();
      }

      // Reseta os acumuladores para a próxima janela de medição
      samplesCollected = 0;
      thrustSum = 0.0;
      adcSum = 0.0;
    }
  }
}