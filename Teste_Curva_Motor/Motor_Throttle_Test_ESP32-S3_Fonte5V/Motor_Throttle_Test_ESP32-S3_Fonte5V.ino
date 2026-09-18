#include <Arduino.h>
#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// --- Definições de Pinos ---
const int HX711_DOUT = 5;
const int HX711_SCK = 6;
const int VOLTAGE_PIN = 4; // ADC1_CH3

// Pinos para o Módulo SD SPI (Padrão sugerido, altere se precisar)
const int SD_CS_PIN = 10;
const int SD_SCK = 12;
const int SD_MISO = 13;
const int SD_MOSI = 11;

// --- Configurações do Ensaio ---
const int SAMPLES_TO_AVERAGE = 10;
const float CALIBRATION_FACTOR = 113.0f;
const float VOLTAGE_DIVIDER_RATIO = 1.0f; // Fator do seu divisor de tensão

HX711 scale;
File dataFile;
char logFileName[32]; 

int samplesCollected = 0;
float thrustSum = 0.0;
float voltageSum = 0.0;

void setup() {
  Serial.begin(115200);
  delay(2000); 
  
  Serial.println("\n--- Setup Datalogger ESP32-S3 ---");

  // 1. Inicializa o barramento SPI customizado para o Cartão SD
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("Falha no modulo SD Card!");
    while (1);
  }
  
  // 2. Indexação de Arquivos
  int fileIndex = 1;
  while (true) {
    sprintf(logFileName, "/ensaio_motor_%03d.csv", fileIndex);
    if (!SD.exists(logFileName)) break; 
    fileIndex++;
  }

  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage_V,Thrust_unit");
    dataFile.close();
    Serial.print("Arquivo criado: "); Serial.println(logFileName);
  } else {
    Serial.println("Erro ao criar o arquivo no SD.");
  }

  // 3. Configura o ADC (ESP32-S3 possui leitura nativa muito estável com eFuse)
  analogSetAttenuation(ADC_11db); // Permite leituras até ~3.1V

  // 4. Inicializa Célula de Carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  Serial.println("Tarando a balança...");
  scale.tare(20); 
  Serial.println("Pronto! Salvando dados continuamente...");
}

void loop() {
  if (scale.is_ready()) {
    float currentThrust = scale.get_units(1);
    
    // Leitura linearizada via eFuse (Padrão recomendado pela Espressif)
    float currentVoltage = (analogReadMilliVolts(VOLTAGE_PIN) / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

    thrustSum += currentThrust;
    voltageSum += currentVoltage;
    samplesCollected++;

    if (samplesCollected >= SAMPLES_TO_AVERAGE) {
      float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
      float avgVoltage = voltageSum / SAMPLES_TO_AVERAGE;
      
      Serial.print("Tensão: "); Serial.print(avgVoltage, 3);
      Serial.print(" V \t| Empuxo: "); Serial.println(avgThrust, 3);
      
      dataFile = SD.open(logFileName, FILE_APPEND); // No ESP32-S3 usa-se FILE_APPEND
      if (dataFile) {
        dataFile.print(avgVoltage, 4); 
        dataFile.print(",");
        dataFile.println(avgThrust, 3); 
        dataFile.close();
      }

      samplesCollected = 0;
      thrustSum = 0.0;
      voltageSum = 0.0;
    }
  }
}