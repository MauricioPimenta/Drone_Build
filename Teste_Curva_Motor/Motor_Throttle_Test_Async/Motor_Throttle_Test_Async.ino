#include <Arduino.h>
#include "HX711.h"   // Biblioteca do Rob Tillaart
#include <Servo.h>
#include <SD.h>
#include <SPI.h>


// --- Definições de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int ESC_PIN = 3;

// --- Configurações do Ensaio ---
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_STEP = 50;
const unsigned long SETTLING_TIME_MS = 1500; // Tempo de acomodação da planta antes de medir (ms)
const int SAMPLES_TO_AVERAGE = 100;           // Quantidade de amostras em regime permanente
const float CALIBRATION_FACTOR = 113.0f;    // Substitua pelo valor encontrado na calibração

HX711 scale;
Servo esc;
File dataFile;

// Variável global para armazenar o nome dinâmico do arquivo
char logFileName[64];

// --- Máquina de Estados do Ensaio ---
enum TestState {
  WAITING_SETTLEMENT, // Aguardando a estabilização da rotação do motor
  ACQUIRING_DATA,     // Coletando amostras de empuxo
  TEST_COMPLETED      // Fim do ensaio
};

TestState currentState = WAITING_SETTLEMENT;
int currentPwm = PWM_MIN;
unsigned long stepStartTime = 0;
int samplesCollected = 0;
float thrustSum = 0.0;



void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("--- Setup de Inicialização ---");

  // 1. Inicialização do SD Card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Falha no SD card! Verifique o hardware.");
    while (1);
  }
  
  // 2. Busca pelo próximo nome de arquivo disponível
  int fileIndex = 1;
  while (true) {
    // Formata o nome iterando o número (ensaio_001_motor_data.txt, ensaio_002...)
    sprintf(logFileName, "ensaio_%03d_motor_data.csv", fileIndex);
    
    // Se o arquivo não existir, encontramos um nome livre. Quebra o loop.
    if (!SD.exists(logFileName)) {
      break; 
    }
    fileIndex++;
    
    // Trava de segurança para não rodar infinito caso o cartão lote
    if (fileIndex > 999) {
      Serial.println("Limite de 999 arquivos atingido!");
      while (1);
    }
  }

  // 3. Cria o arquivo com o nome inédito e escreve o cabeçalho
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
  sleep(1);
  Serial.print("1... ");
  sleep(1);
  Serial.print("2... ");
  sleep(1);
  Serial.println("3... ");
  
  // Realiza a tara tirando uma média de 20 amostras no setup (bloqueante, mas OK aqui)
  scale.tare(20); 
  Serial.println("Balança zerada.");

  // 3. Inicialização e Armação do ESC
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(PWM_MIN);
  Serial.println("Aguardando 5s para o ESC reconhecer o sinal mínimo (bipes)...");
  delay(5000);
  
  Serial.println("ESC Armado. Iniciando ensaio de degraus de carga...");
  
  // Aplica o primeiro degrau (ou condição inicial) e marca o tempo
  esc.writeMicroseconds(currentPwm);
  stepStartTime = millis();
}







void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    
    case WAITING_SETTLEMENT:
      // Aguarda a resposta transitória da hélice passar (tempo de acomodação)
      if (currentMillis - stepStartTime >= SETTLING_TIME_MS) {
        currentState = ACQUIRING_DATA;
        samplesCollected = 0;
        thrustSum = 0.0;
      }
      break;

    case ACQUIRING_DATA:
      // Verifica se o HX711 tem uma conversão pronta (Não-bloqueante)
      if (scale.is_ready()) {
        thrustSum += scale.get_units(1); // O argumento (1) lê uma única amostra atualizada
        samplesCollected++;

        // Quando coletar amostras suficientes no regime permanente
        if (samplesCollected >= SAMPLES_TO_AVERAGE) {
          float avgThrust = thrustSum / SAMPLES_TO_AVERAGE;
          
          // Log no Monitor Serial
          Serial.print("PWM: ");
          Serial.print(currentPwm);
          Serial.print(" us \t| Empuxo Médio: ");
          Serial.println(avgThrust, 3);
          
          // Log no SD Card
          dataFile = SD.open(logFileName, FILE_WRITE);
          if (dataFile) {
            dataFile.print(currentPwm);
            dataFile.print(",");
            dataFile.println(avgThrust, 4); 
            dataFile.close();
          }

          // Prepara o próximo degrau
          currentPwm += PWM_STEP;
          
          // Checa condição de parada do ensaio
          if (currentPwm > PWM_MAX) {
            esc.writeMicroseconds(PWM_MIN); // Corta a potência imediatamente
            currentState = TEST_COMPLETED;
            Serial.println("Ensaio concluído com sucesso. Os motores foram desativados.");
          } else {
            // Aplica novo degrau, reseta o timer e volta para a fase de estabilização
            esc.writeMicroseconds(currentPwm); 
            stepStartTime = currentMillis;     
            currentState = WAITING_SETTLEMENT; 
          }
        }
      }
      break;

    case TEST_COMPLETED:
      // O ensaio terminou. O microcontrolador fica ocioso para segurança 
      // ou pode ser usado para fechar o arquivo e comunicar fim de script.
      break;
  }
}