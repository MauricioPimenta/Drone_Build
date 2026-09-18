#include <Arduino.h>
#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// --- Definições de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int VOLTAGE_PIN = A0;  // Leitura da tensão aplicada ao ESC
const int SD_CS_PIN = 10;    // Pino CS do módulo de cartão SD

// --- Configurações do Ensaio ---
const int SAMPLES_TO_AVERAGE = 10;        // Quantidade de amostras para cada média
const float CALIBRATION_FACTOR = 113.0f; // Fator calibrado do HX711

// --- Configurações do ADC (Arduino Uno) ---
const float ADC_MAX_VAL = 1023.0f;        // Resolução de 10 bits (2^10 - 1)
const float VREF = 5.0f;                  // Tensão lógica nativa do Arduino Uno

/* 
 * FATOR DO DIVISOR DE TENSÃO
 * O pino A0 do Uno tolera no MÁXIMO 5V. Se a sua fonte do ESC for de 12V, 
 * por exemplo, você DEVE usar um divisor resistivo (R1 e R2).
 * VOLTAGE_DIVIDER_RATIO = (R1 + R2) / R2
 * Se a fonte não passar de 5V e for ligada direto no A0, deixe 1.0f.
 */
const float VOLTAGE_DIVIDER_RATIO = 1.0f; 

HX711 scale;
File dataFile;
char logFileName[13]; // Buffer para formato FAT 8.3: "DATA_001.TXT" + null terminator

// --- Variáveis de Acúmulo e Médias ---
int samplesCollected = 0;
float thrustSum = 0.0;
float adcSum = 0.0;

void setup() {
  Serial.begin(115200);
  // No Uno, não precisamos do while(!Serial) pois a USB não é nativa no processador.
  
  Serial.println("--- Setup de Inicialização (Arduino Uno Datalogger) ---");

  // 1. Inicialização do SD Card e Indexação
  Serial.print("Inicializando SD Card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(" Falha! Verifique conexoes e CS.");
    while (1);
  }
  Serial.println(" OK!");
  
  // Busca pelo próximo nome de arquivo disponível (Padrão 8.3)
  int fileIndex = 1;
  while (true) {
    sprintf(logFileName, "DATA_%03d.csv", fileIndex);
    if (!SD.exists(logFileName)) {
      break; 
    }
    fileIndex++;
    if (fileIndex > 999) {
      Serial.println("Limite de arquivos atingido no SD!");
      while (1);
    }
  }

  // Cria e formata o cabeçalho CSV
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage_V,Thrust_unit");
    dataFile.close();
    Serial.print("Novo arquivo de log criado: ");
    Serial.println(logFileName);
  } else {
    Serial.println("Erro ao criar o arquivo no SD.");
  }

  // 2. Inicialização da Célula de Carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  
  Serial.println("Tarando a balança... Remova qualquer peso do motor.");
  scale.tare(20); 
  Serial.println("Balança zerada.");
  
  Serial.println("=== SISTEMA PRONTO ===");
  Serial.println("Você já pode começar a variar a tensão na fonte do ESC.");
  Serial.println("Salvando dados continuamente...");
}

void loop() {
  // Amostragem controlada pelo ritmo de conversão do HX711 (aprox 10Hz/80Hz)
  if (scale.is_ready()) {
    
    // 1. Lê a força e a tensão sincronizadas
    float currentThrust = scale.get_units(1);
    float currentAdc = analogRead(VOLTAGE_PIN); // Retorna de 0 a 1023

    // 2. Acumula os valores
    thrustSum += currentThrust;
    adcSum += currentAdc;
    samplesCollected++;

    // 3. Processa e salva a média a cada janela de amostras
    if (samplesCollected >= SAMPLES_TO_AVERAGE) {
      
      float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
      float avgAdc = adcSum / SAMPLES_TO_AVERAGE;
      
      // Conversão matemática: ADC (10-bits) -> Tensão Real
      float actualVoltage = (avgAdc / ADC_MAX_VAL) * VREF * VOLTAGE_DIVIDER_RATIO;
      
      // Log no Monitor Serial
      Serial.print("Tensão Aplicada: ");
      Serial.print(actualVoltage, 3);
      Serial.print(" V \t| Empuxo: ");
      Serial.println(avgThrust, 3);
      
      // Log no Cartão SD (Garante gravação imediata no disco)
      dataFile = SD.open(logFileName, FILE_WRITE);
      if (dataFile) {
        dataFile.print(actualVoltage, 4); 
        dataFile.print(",");
        dataFile.println(avgThrust, 3); 
        dataFile.close();
      }

      // Reseta para a próxima janela
      samplesCollected = 0;
      thrustSum = 0.0;
      adcSum = 0.0;
    }
  }
}