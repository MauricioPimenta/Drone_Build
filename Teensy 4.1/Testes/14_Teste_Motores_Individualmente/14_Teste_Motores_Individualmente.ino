/*
  This code sends commands to the motors individually to test their starting pulse in microseconds.
  As we checked, the ESCs stop beeping and start rotating the motor at the following pulse time:
  
  Motor Dianteiro Esquerdo (Mde) or Front-Left Motor (Mfl):
  - stop beeping: ~1085us
  - Start Spinning: 1393

  Motor Dianteiro Direito (Mdd) or Front-Right Motor (Mfr):
  - stop beebinp: ~1085us
  - Start spinning: 1427us

  Motor Traseiro Esquerdo (Mte) or Back-Left Motor (Mbl):
  - stop beeping: ~1085us
  - Start Spinning: 1441us

  Motor Dianteiro Direito (Mtd) or Back-Right Motor (Mbr):
  - stop beebinp: 1405~1415us
  - Start spinning: 1769us


  Signal logic is inverted:
    Teensy LOW  => transistor OFF => ESC signal pulled HIGH (by ESC pull-up to 5V)
    Teensy HIGH => transistor ON  => ESC signal LOW
*/

#include <Arduino.h>

/* 
 * Motor Pins are now soldered in the PCB
 * Pin 3 = Motor traseiro direito
 * Pin 4 = Motor traseiro esquerdo
 * Pin 5 = Motor dianteiro esquerdo
 * Pin 6 = Motor dianteiro direito
 */
const int motor_traseiro_direito = 3;       // choose a digital pin for ESC control (any normal GPIO works)
const int motor_traseiro_esquerdo = 4;
const int motor_dianteiro_esquerdo = 5;
const int motor_dianteiro_direito = 6;

// const int LED_red = 0;
// const int LED_green = 1;
// const int LED_blue = 2;


constexpr uint32_t PERIOD_US = 20000;  // 50 Hz
constexpr int PULSE_MIN_US = 1000;     // typical ESC min
constexpr int PULSE_MAX_US = 2000;     // typical ESC max

// Helps arming: keep minimum throttle at boot
constexpr uint32_t ARM_TIME_MS = 3000;

// Filtering (simple IIR low-pass)
// Bigger SHIFT => more smoothing (slower response).
// SHIFT=4 => alpha = 1/16 (good start)
constexpr int FILTER_SHIFT = 4;

// Optional deadband near minimum to prevent tiny noise from spinning motor
constexpr int DEAD_BAND_US = 20;

// If you want to limit max throttle (safety), reduce this (e.g. 1700)
constexpr int SAFETY_MAX_US = PULSE_MAX_US;

// ----- Internal state -----
elapsedMillis tick;
int filteredPulseUs = PULSE_MIN_US;

int raw = 0;
bool inc = true;

// Variaveis pra ler do Serial usando char
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Motor ativo:
// 0 = Motor Dianteiro Esquerdo;
// 1 = Motor Dianteiro Direito;
// 2 = Motor Traseiro Esquerdo;
// 3 = Motor Traseiro Direito;
int motor_ativo = 0;

// Tempo do pulso para controle do motor
// Motor Dianteiro Esquerdo (Mde) or Front-Left Motor (Mfl):
//   - stop beeping: ~1085us
//   - Start Spinning: 1393
//
//   Motor Dianteiro Direito (Mdd) or Front-Right Motor (Mfr):
//   - stop beebinp: ~1085us
//   - Start spinning: 1427us
//
//   Motor Traseiro Esquerdo (Mte) or Back-Left Motor (Mbl):
//   - stop beeping: ~1085us
//   - Start Spinning: 1441us
//
//   Motor Dianteiro Direito (Mtd) or Back-Right Motor (Mbr):
//   - stop beebinp: 1415us
//   - Start spinning: 1769us
int pulse = 1000;

int targetPulseUs;

/*
 * FUNÇÕES DO PROGRAMA
 */
void sendEscPulseInverted(int high_us);
void processarComando(char *cmd);
void lerSerialNaoBloqueante();





/* ---- SETUP  ----*/
void setup() {

  Serial.begin(115200);

  // Define pin type as INPUT or OUTPUT
  pinMode(motor_dianteiro_esquerdo, OUTPUT);
  pinMode(motor_dianteiro_direito, OUTPUT);
  
  pinMode(motor_traseiro_esquerdo, OUTPUT);
  pinMode(motor_traseiro_direito, OUTPUT);
  
  
  // Start with LOW = ESC is HIGH
  digitalWriteFast(motor_dianteiro_esquerdo, LOW);
  digitalWriteFast(motor_dianteiro_direito, LOW);
  
  digitalWriteFast(motor_traseiro_esquerdo, LOW);
  digitalWriteFast(motor_traseiro_direito, LOW);

  // LED
  // pinMode(LED_red, OUTPUT);
  // pinMode(LED_green, OUTPUT);
  // pinMode(LED_blue, OUTPUT);

  // digitalWrite(LED_red, HIGH);

  // Wait for Serial to be ready
  while (!Serial){delay(10);}
  Serial.println("Sistema iniciado.");
  Serial.println("Comandos disponiveis:");
  Serial.println("  M <valor>    -> define motor: 0 = Mde; 1 = Mdd; 2 = Mte; 3 = Mtd");
  // Serial.println("  v <valor>    -> define velocidade");
  // Serial.println("  g <valor>    -> define ganho");
  Serial.println("  status       -> mostra valores atuais");

  // Let things power up
  delay(5000);

  // Arm ESC at minimum throttle for a while
  uint32_t t0 = millis();
  uint i = 0;
  while (millis() - t0 < ARM_TIME_MS) {
    Serial.print("0 - ");
    Serial.println(i);
    sendEscPulseInverted(PULSE_MIN_US, 0);
    i++;
  }

  t0 = millis();
  i = 0;
  while (millis() - t0 < ARM_TIME_MS) {
    Serial.print("1 - ");
    Serial.println(i);
    sendEscPulseInverted(PULSE_MIN_US, 1);
    i++;
  }
  
  t0 = millis();
  i = 0;
  while (millis() - t0 < ARM_TIME_MS) {
    Serial.print("2 - ");
    Serial.println(i);
    sendEscPulseInverted(PULSE_MIN_US, 2);
    i++;
  }

  t0 = millis();
  i = 0;
  while (millis() - t0 < ARM_TIME_MS) {
    Serial.print("3 - ");
    Serial.println(i);
    sendEscPulseInverted(PULSE_MIN_US, 3);
    i++;
  }

  // Initialize filter at min
  filteredPulseUs = PULSE_MIN_US;
}


/* ---- LOOP ---- */
void loop() {

  // Ler serial sem travar o programa
  lerSerialNaoBloqueante();

  // 2) Map to pulse width
  if (targetPulseUs > pulse)
  {
    filteredPulseUs = PULSE_MIN_US;
  }
  
  targetPulseUs = pulse;
  Serial.print("Pulse: ");
  Serial.println(targetPulseUs);

  // 3) Deadband near min (helps prevent unintended spin from noise)
  if (targetPulseUs < (PULSE_MIN_US + DEAD_BAND_US))
    targetPulseUs = PULSE_MIN_US;

  
  // 4) IIR filter: filtered += (target - filtered)/2^SHIFT
  filteredPulseUs += (targetPulseUs - filteredPulseUs) >> FILTER_SHIFT;
  Serial.println(filteredPulseUs);

  // 5) Output one ESC frame (20ms total)
  sendEscPulseInverted(filteredPulseUs, motor_ativo);

  // loop repeats at ~50Hz because sendEscPulseInverted already delays ~20ms
  
}



/**************************************************************************************
 *
 * FUNCTIONS
 *
 **************************************************************************************/

/*
 * Inverted pulse generator: keeps ESC line HIGH for high_us, then LOW for rest of period.
 */
void sendEscPulseInverted(int high_us, int motor) {
  high_us = constrain(high_us, PULSE_MIN_US, SAFETY_MAX_US);

  // ESC line HIGH:
  // (inverted) -> Teensy LOW turns transistor OFF -> ESC pull-up makes line HIGH
  switch(motor_ativo){
    case 0:
    {
      digitalWriteFast(motor_dianteiro_esquerdo, LOW);
      Serial.println("000000000000000000000 M0 enviado 00000000000000000000000000");
      delayMicroseconds(high_us);
      digitalWriteFast(motor_dianteiro_esquerdo, HIGH);
      delayMicroseconds((int)PERIOD_US - high_us);
      break;
    }
    case 1:
    {
      digitalWriteFast(motor_dianteiro_direito, LOW);
      Serial.println("111111111111111111 M1 enviado 1111111111111111111111");
      delayMicroseconds(high_us);
      digitalWriteFast(motor_dianteiro_direito, HIGH);
      delayMicroseconds((int)PERIOD_US - high_us);
      break;
    }
    case 2:
    {
      digitalWriteFast(motor_traseiro_esquerdo, LOW);
      Serial.println("22222222222222222222222222222 M0 enviado 22222222222222222222222222222");
      delayMicroseconds(high_us);
      digitalWriteFast(motor_traseiro_esquerdo, HIGH);
      delayMicroseconds((int)PERIOD_US - high_us);
      break;
    }
    case 3:
    {
      digitalWriteFast(motor_traseiro_direito, LOW);
      Serial.println("333333333333333333333333 M3 enviado 333333333333333333333333333");
      delayMicroseconds(high_us);
      digitalWriteFast(motor_traseiro_direito, HIGH);
      delayMicroseconds((int)PERIOD_US - high_us);
      break;
    }
    default:
      Serial.println("Motor Invalido - comando nao enviado");
      return;
  }
  
}

void processarComando(char *cmd) {
  // Remove espaços iniciais
  while (*cmd == ' ') cmd++;

  if (strncmp(cmd, "M ", 2) == 0) {
    motor_ativo = atoi(cmd + 2);
    if (motor_ativo < 0 || motor_ativo > 4)
    Serial.println("Motor Invalido!!");
    else{
      Serial.print("Motor atualizado para: M");
      Serial.println(motor_ativo);
    }
  }
  else if (strncmp(cmd, "p ", 2) == 0) {
    pulse = atoi(cmd + 2);
    Serial.print("Pulso atualizado para: ");
    Serial.println(pulse);
  }
  else if (strcmp(cmd, "status") == 0) {
    Serial.println("=== STATUS ===");
    Serial.print("Motor: M");
    Serial.print(motor_ativo);
    Serial.print("Pulse: ");
    Serial.println(pulse, 4);
  }
  else {
    Serial.print("Comando invalido: ");
    Serial.println(cmd);
    Serial.println("Use:");
    Serial.println("  M <valor>");
    Serial.println("  p <valor>");
    Serial.println("  status");
  }
}

void lerSerialNaoBloqueante() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Ignora carriage return
    if (c == '\r') {
      continue;
    }

    // Quando pressiona ENTER, processa a linha
    if (c == '\n') {
      buffer[bufferIndex] = '\0';

      if (bufferIndex > 0) {
        processarComando(buffer);
      }

      bufferIndex = 0;  // limpa buffer para próximo comando
    }
    else {
      // Armazena caractere se houver espaço
      if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = c;
      }
      else {
        // Buffer cheio: descarta e reinicia
        Serial.println("Erro: comando muito longo.");
        bufferIndex = 0;
      }
    }
  }
}