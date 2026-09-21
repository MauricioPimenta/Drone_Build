#include <Arduino.h>
#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// --- Definicoes de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int ESC_PIN = 3; 

// --- Configuracoes do Ensaio ---
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_STEP = 50;
const unsigned long SETTLING_TIME_MS = 1500; 
const int SAMPLES_TO_AVERAGE = 10;           
const float CALIBRATION_FACTOR = 113.0f;    

// --- Configuracoes de PWM Nativo da Teensy ---
const uint32_t PWM_FREQUENCY = 50; // Hz
const float PERIOD_US = 1000000.0 / PWM_FREQUENCY; // 20000 us (20 ms)
const int PWM_RESOLUTION_BITS = 12;  // Resolucao configuravel (ex: 8, 10, 12, 14, 16...)

// Calcula automaticamente 2^Bits em tempo de compilacao (ex: 1 << 12 = 4096)
const float MAX_DUTY_VAL = (float)(1 << PWM_RESOLUTION_BITS);

// IMPORTANTE: Altere para 'false' se um dia ligar o sinal da Teensy direto no ESC (sem transistor)
const bool INVERT_ESC_SIGNAL = true; 

HX711 scale;
File dataFile;
char logFileName[64]; 

// --- Maquina de Estados do Ensaio ---
enum TestState {
  WAITING_SETTLEMENT,
  ACQUIRING_DATA,    
  TEST_COMPLETED      
};

TestState currentState = WAITING_SETTLEMENT;
int currentPwm = PWM_MIN;
unsigned long stepStartTime = 0;
int samplesCollected = 0;
float thrustSum = 0.0;


// ====================================================================
// FUNCAO PARA CONTROLE DO ESC VIA HARDWARE PWM
// ====================================================================
void setEscPulse(int pulseUs) {
  uint32_t duty;
  
  if (INVERT_ESC_SIGNAL) {
    // Se há transistor inversor: 
    // Para o ESC ver HIGH por 'pulseUs', a Teensy precisa ficar LOW por 'pulseUs'
    // Logo, o tempo em HIGH da Teensy é o restante do período.
    float highTimeUs = PERIOD_US - pulseUs;
    duty = (highTimeUs / PERIOD_US) * MAX_DUTY_VAL;
  } else {
    // Ligação direta padrão
    duty = (pulseUs / PERIOD_US) * MAX_DUTY_VAL;
  }
  
  analogWrite(ESC_PIN, duty);
}

// ====================================================================


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("--- Setup de Inicialização ---");

  // 1. Configuração do Hardware PWM da Teensy
  analogWriteFrequency(ESC_PIN, PWM_FREQUENCY);
  analogWriteResolution(PWM_RESOLUTION_BITS);

  // 2. Inicialização do SD Card com Indexação Sequencial
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Falha no SD card! Verifique o hardware.");
    while (1);
  }
  
  int fileIndex = 1;
  while (true) {
    sprintf(logFileName, "ensaio_%03d_motor_data.csv", fileIndex);
    if (!SD.exists(logFileName)) {
      break; 
    }
    fileIndex++;
    if (fileIndex > 999) while (1); // Trava de segurança
  }

  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("PWM_us,Thrust_unit");
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

  // 4. Inicialização e Armação do ESC
  setEscPulse(PWM_MIN); // Garante que começa no pulso mínimo
  Serial.println("Aguardando 5s para o ESC reconhecer o sinal mínimo (bipes)...");
  delay(5000);
  
  Serial.println("ESC Armado. Iniciando ensaio de degraus de carga...");
  setEscPulse(currentPwm);
  stepStartTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    
    case WAITING_SETTLEMENT:
      if (currentMillis - stepStartTime >= SETTLING_TIME_MS) {
        currentState = ACQUIRING_DATA;
        samplesCollected = 0;
        thrustSum = 0.0;
      }
      break;

    case ACQUIRING_DATA:
      if (scale.is_ready()) {
        thrustSum += scale.get_units(1); 
        samplesCollected++;

        if (samplesCollected >= SAMPLES_TO_AVERAGE) {
          float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
          
          Serial.print("PWM: ");
          Serial.print(currentPwm);
          Serial.print(" us \t| Empuxo Médio: ");
          Serial.println(avgThrust, 3);
          
          dataFile = SD.open(logFileName, FILE_WRITE);
          if (dataFile) {
            dataFile.print(currentPwm);
            dataFile.print(",");
            dataFile.println(avgThrust, 3); 
            dataFile.close();
          }

          currentPwm += PWM_STEP;
          
          if (currentPwm > PWM_MAX) {
            setEscPulse(PWM_MIN); // Corta potência
            currentState = TEST_COMPLETED;
            Serial.println("Ensaio concluído com sucesso. O motor foi desativado.");
          } else {
            setEscPulse(currentPwm); // Atualiza potência
            stepStartTime = currentMillis;     
            currentState = WAITING_SETTLEMENT; 
          }
        }
      }
      break;

    case TEST_COMPLETED:
      // Fica ocioso com segurança
      break;
  }
}