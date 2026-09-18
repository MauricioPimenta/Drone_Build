#include <Arduino.h>
#include <HX711.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>

// --- Definições de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;
const int ESC_PIN = 3;

// --- Configurações do Ensaio ---
const int PWM_MIN = 1000;      // Sinal de braço/parada do ESC (µs)
const int PWM_MAX = 2000;      // Aceleração máxima (µs)
const int PWM_STEP = 50;       // Incremento do degrau de PWM
const int STEP_DELAY = 1500;   // Tempo de estabilização do motor antes da leitura (ms)
const float CALIBRATION_FACTOR = 113.0f; // Ajuste com um peso conhecido previamente

HX711 scale;
Servo esc;
File dataFile;

bool testCompleted = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); 

  // 1. Inicialização do Cartão SD (Usando o slot nativo da Teensy 4.1)
  Serial.print("Inicializando SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Falha no SD card! Verifique a conexão.");
    while (1); 
  }
  Serial.println("SD inicializado.");

  // Cria/abre o arquivo e escreve o cabeçalho (formato CSV para fácil importação)
  dataFile = SD.open("motor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("PWM_us,Thrust_unit");
    dataFile.close();
  } else {
    Serial.println("Erro ao abrir motor_data.txt");
  }

  // 2. Inicialização da Célula de Carga
  Serial.println("Inicializando HX711...");
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare(); // Zera a balança considerando o peso inicial do motor/hélice
  Serial.println("Balança zerada.");

  // 3. Inicialização e Armação do ESC
  Serial.println("Armando o ESC...");
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(PWM_MIN);
  delay(5000); // Aguarda 5s para o ESC reconhecer o sinal de zero e emitir os bipes
  Serial.println("ESC armado. Iniciando ensaio da curva de carga em 3 segundos...");
  delay(3000);
}

void loop() {
  if (!testCompleted) {
    
    // Varredura de PWM do mínimo ao máximo
    for (int pwm = PWM_MIN; pwm <= PWM_MAX; pwm += PWM_STEP) {
      
      // Atualiza o setpoint do motor
      esc.writeMicroseconds(pwm);
      
      // Aguarda a resposta transitória do conjunto motor-hélice passar
      delay(STEP_DELAY);
      
      // Realiza a leitura da força (Média de 5 amostras para atenuar o ruído de vibração)
      float thrust = scale.get_units(5);
      
      // Log no Serial
      Serial.print("PWM: ");
      Serial.print(pwm);
      Serial.print(" us \t| Empuxo: ");
      Serial.println(thrust);
      
      // Log no Cartão SD
      dataFile = SD.open("motor_data.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.print(pwm);
        dataFile.print(",");
        dataFile.println(thrust);
        dataFile.close();
      } else {
        Serial.println("Erro de I/O no SD.");
      }
    }

    // Fim do ensaio: Corta a potência do motor imediatamente
    esc.writeMicroseconds(PWM_MIN);
    Serial.println("Ensaio concluído. Dados salvos no cartão SD.");
    testCompleted = true;
  }
}
