#include <Arduino.h>
#include "HX711.h"
#include <SD_MMC.h> // Biblioteca específica para o leitor embutido da ESP32-CAM

// --- Definições de Pinos (Mapeamento crítico para ESP32-CAM) ---
const int HX711_DOUT = 16;   // Pino livre (geralmente usado para U2RX)
const int HX711_SCK = 4;     // Liberado pelo SD 1-bit (Aviso: O LED Flash pode piscar sutilmente)
const int VOLTAGE_PIN = 13;  // ADC2_CH4 - Liberado pelo SD 1-bit

// --- Configurações do Ensaio ---
const int SAMPLES_TO_AVERAGE = 10;
const float CALIBRATION_FACTOR = 113.0f;
const float VOLTAGE_DIVIDER_RATIO = 1.0f; // Fator do seu divisor de tensão (se houver)

HX711 scale;
File dataFile;
char logFileName[32]; 

int samplesCollected = 0;
float thrustSum = 0.0;
float voltageSum = 0.0; // Agora somaremos a tensão real já convertida

void setup() {
  Serial.begin(115200);
  delay(1000); // Dá tempo para o monitor serial abrir no CH340
  
  Serial.println("\n--- Setup Datalogger ESP32-CAM ---");

  // 1. Inicializa SD Card em modo 1-bit (O argumento 'true' é fundamental!)
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Falha ao montar o SD Card!");
    while (1);
  }
  
  // 2. Indexação de Arquivos
  int fileIndex = 1;
  while (true) {
    sprintf(logFileName, "/curva_%03d.csv", fileIndex);
    if (!SD_MMC.exists(logFileName)) break; 
    fileIndex++;
  }

  dataFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage_V,Thrust_unit");
    dataFile.close();
    Serial.print("Arquivo criado: "); Serial.println(logFileName);
  } else {
    Serial.println("Erro de I/O no SD.");
  }

  // 3. Inicializa ADC (Ajusta atenuação para ler até ~3.1V/3.3V)
  analogSetAttenuation(ADC_11db);

  // 4. Inicializa Célula de Carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  Serial.println("Tarando balança...");
  scale.tare(20); 
  Serial.println("Pronto! Iniciando leituras...");
}

void loop() {
  if (scale.is_ready()) {
    float currentThrust = scale.get_units(1);
    
    // Lê a tensão em milivolts usando a calibração de fábrica do ESP32 e converte para Volts
    float currentVoltage = (analogReadMilliVolts(VOLTAGE_PIN) / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

    thrustSum += currentThrust;
    voltageSum += currentVoltage;
    samplesCollected++;

    if (samplesCollected >= SAMPLES_TO_AVERAGE) {
      float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
      float avgVoltage = voltageSum / SAMPLES_TO_AVERAGE;
      
      Serial.print("Tensão: "); Serial.print(avgVoltage, 3);
      Serial.print(" V \t| Empuxo: "); Serial.println(avgThrust, 3);
      
      dataFile = SD_MMC.open(logFileName, FILE_APPEND);
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