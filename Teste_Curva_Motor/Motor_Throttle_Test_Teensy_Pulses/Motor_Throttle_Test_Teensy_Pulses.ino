#include <Arduino.h>
#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// --- Definicoes de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int ESC_PIN = 3; 

// --- Configuracoes do Ensaio (Atualizadas via GUI) ---
int pwmMin = 1000;
int pwmMax = 2000;
int pwmStep = 50;
const unsigned long SETTLING_TIME_MS = 1500; 
const int SAMPLES_TO_AVERAGE = 10;           
const float CALIBRATION_FACTOR = 113.0f;    

// --- Configuracoes de PWM Nativo da Teensy ---
const uint32_t PWM_FREQUENCY = 50; // 50 Hz (periodo de 20 ms)
const float PERIOD_US = 1000000.0 / PWM_FREQUENCY; 
const int PWM_RESOLUTION_BITS = 12;  
const float MAX_DUTY_VAL = (float)(1 << PWM_RESOLUTION_BITS);

// Inversor de sinal via transistor na sua PCB personalizada
const bool INVERT_ESC_SIGNAL = true; 

// --- Instancias e Flags ---
HX711 scale;
File dataFile;
char logFileName[64]; 
bool sdCardPresent = false; // FLAG: Permite operar com ou sem cartao SD

// --- Maquina de Estados do Ensaio ---
enum TestState {
  STATE_IDLE,
  WAITING_SETTLEMENT,
  ACQUIRING_DATA,    
  TEST_COMPLETED      
};

TestState currentState = STATE_IDLE;
int currentPwm = 1000;
unsigned long stepStartTime = 0;
int samplesCollected = 0;
float thrustSum = 0.0;
float currentThrustInstant = 0.0;
bool startTest = false;

// Controle de taxa de transmissao para a GUI (20 Hz = 50 ms)
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// ====================================================================
// CONTROLE DO ESC VIA HARDWARE PWM
// ====================================================================
void setEscPulse(int pulseUs) {
  uint32_t duty;
  
  if (INVERT_ESC_SIGNAL) {
    float highTimeUs = PERIOD_US - pulseUs;
    duty = (highTimeUs / PERIOD_US) * MAX_DUTY_VAL;
  } else {
    duty = (pulseUs / PERIOD_US) * MAX_DUTY_VAL;
  }
  
  analogWrite(ESC_PIN, duty);
}

// ====================================================================
// PREPARACAO E INICIO DO ENSAIO
// ====================================================================
void iniciarEnsaio() {
  Serial.println("# Iniciando rotina de preparacao do ensaio...");

  // 1. Gerencia criacao do arquivo de log no SD se disponivel
  if (sdCardPresent) {
    int fileIndex = 1;
    while (fileIndex <= 999) {
      sprintf(logFileName, "ensaio_%03d_motor_data.csv", fileIndex);
      if (!SD.exists(logFileName)) {
        break; 
      }
      fileIndex++;
    }

    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("PWM_us,Thrust_unit");
      dataFile.close();
      Serial.print("# Novo arquivo de log criado: ");
      Serial.println(logFileName);
    } else {
      Serial.println("# Erro ao criar arquivo no SD. Continuando sem gravar em disco.");
    }
  } else {
    Serial.println("# Cartao SD desabilitado. Operando em modo de telemetria apenas.");
  }

  // 2. Tara da Balanca
  Serial.println("# Tarando a balanca... Remova qualquer peso do motor.");
  scale.tare(20); 
  Serial.println("# Balanca zerada com sucesso.");

  // 3. Armacao do ESC no pulso minimo configurado
  currentPwm = pwmMin;
  setEscPulse(pwmMin);
  Serial.println("# Aguardando 3s para o ESC reconhecer o sinal minimo...");
  delay(3000);
  
  Serial.println("# ESC Armado. Iniciando ensaio de degraus de carga!");
  setEscPulse(currentPwm);
  stepStartTime = millis();
  samplesCollected = 0;
  thrustSum = 0.0;
  currentState = WAITING_SETTLEMENT;
  startTest = true;
}

// ====================================================================
// PARADA / CORTE DE SEGURANCA
// ====================================================================
void pararEnsaio() {
  currentPwm = pwmMin;
  setEscPulse(pwmMin);
  currentState = TEST_COMPLETED;
  startTest = false;
  Serial.println("# Ensaio finalizado ou cancelado. Motor em pulso minimo.");
}

// ====================================================================
// PARSER DE COMANDOS VINDOS DA GUI
// ====================================================================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // A GUI envia: "START,min,max,step" (ex: "START,1000,2000,50")
    if (cmd.startsWith("START")) {
      int pMin, pMax, pStep;
      int parsed = sscanf(cmd.c_str(), "START,%d,%d,%d", &pMin, &pMax, &pStep);
      
      if (parsed == 3) {
        pwmMin = pMin;
        pwmMax = pMax;
        pwmStep = pStep;
        Serial.printf("# Parametros recebidos da GUI: Min=%d us, Max=%d us, Step=%d us\n", pwmMin, pwmMax, pwmStep);
      } else {
        Serial.println("# Comando START recebido sem parametros. Usando valores padrao.");
      }
      
      iniciarEnsaio();
    } 
    else if (cmd.equalsIgnoreCase("STOP")) {
      pararEnsaio();
    }
  }
}

// ====================================================================
// SETUP
// ====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("# --- Teensy 4.1 Test Stand Inicializado ---");

  // 1. Hardware PWM
  analogWriteFrequency(ESC_PIN, PWM_FREQUENCY);
  analogWriteResolution(PWM_RESOLUTION_BITS);
  setEscPulse(pwmMin); // Garante sinal baixo ao ligar

  // 2. Inicializacao do SD Card (Sem travar o codigo se falhar)
  if (SD.begin(BUILTIN_SDCARD)) {
    sdCardPresent = true;
    Serial.println("# SD Card inicializado com sucesso.");
  } else {
    sdCardPresent = false;
    Serial.println("# AVISO: SD Card nao encontrado! Operando sem gravacao local.");
  }

  // 3. Inicializacao do Sensor de Carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);

  Serial.println("# Sistema em espera. Envie START pela interface para iniciar.");
}

// ====================================================================
// LOOP PRINCIPAL
// ====================================================================
void loop() {
  // Verifica se a GUI enviou comando START ou STOP
  checkSerialCommands();

  unsigned long currentMillis = millis();

  // Leitura continua da balanca (se houver dado pronto)
  if (scale.is_ready()) {
    currentThrustInstant = scale.get_units(1);
  }

  // Maquina de Estados do Ensaio
  if (startTest) {
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
          thrustSum += currentThrustInstant; 
          samplesCollected++;

          if (samplesCollected >= SAMPLES_TO_AVERAGE) {
            float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
            
            // Grava no cartao SD apenas se a flag estiver ativa
            if (sdCardPresent) {
              dataFile = SD.open(logFileName, FILE_WRITE);
              if (dataFile) {
                dataFile.print(currentPwm);
                dataFile.print(",");
                dataFile.println(avgThrust, 3); 
                dataFile.close();
              }
            }

            // Avanca para o proximo degrau
            currentPwm += pwmStep;
            
            if (currentPwm > pwmMax) {
              pararEnsaio();
              Serial.println("# Ensaio concluido com sucesso!");
            } else {
              setEscPulse(currentPwm);
              stepStartTime = currentMillis;     
              currentState = WAITING_SETTLEMENT; 
            }
          }
        }
        break;

      case TEST_COMPLETED:
        setEscPulse(pwmMin);
        startTest = false;
        break;

      case STATE_IDLE:
      default:
        break;
    }
  }

  // -------------------------------------------------------------
  // Transmissao de Telemetria para a GUI a 20 Hz
  // Protocolo: tempo_ms,pulso_us,thrust_g,estado
  // -------------------------------------------------------------
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;

    const char* stateStr = "IDLE";
    if (currentState == WAITING_SETTLEMENT) stateStr = "waiting Settlement";
    else if (currentState == ACQUIRING_DATA) stateStr = "Acquiring Data";
    else if (currentState == TEST_COMPLETED) stateStr = "Test Completed";

    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(currentPwm);
    Serial.print(",");
    Serial.print(currentThrustInstant, 2);
    Serial.print(",");
    Serial.println(stateStr);
  }
}