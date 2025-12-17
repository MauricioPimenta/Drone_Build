#include <SoftwareSerial.h>

// --- Motor 1 ---
const int SOFT_RX_PIN_1 = 4;
const int SOFT_TX_PIN_1 = 5;
SoftwareSerial mySerial(SOFT_RX_PIN_1, SOFT_TX_PIN_1);

// --- Motor 2 ---
const int SOFT_RX_PIN_2 = 6;
const int SOFT_TX_PIN_2 = 7;
SoftwareSerial mySerial2(SOFT_RX_PIN_2, SOFT_TX_PIN_2);

// --- Motor 3 ---
const int SOFT_RX_PIN_3 = 8;
const int SOFT_TX_PIN_3 = 9;
SoftwareSerial mySerial3(SOFT_RX_PIN_3, SOFT_TX_PIN_3);

// --- Motor 4 ---
const int SOFT_RX_PIN_4 = 10;
const int SOFT_TX_PIN_4 = 11;
SoftwareSerial mySerial4(SOFT_RX_PIN_4, SOFT_TX_PIN_4);

// Potenciômetro
const int POT_PIN = A0;
// Faixa útil do pot (ajuste se necessário)
const int POT_MIN = 20;    // leitura mínima considerada (ruído abaixo disso ignora)
const int POT_MAX = 1023;  // leitura máxima considerada

// Janela de idle (pot bem baixo -> motores OFF/idle)
const int POT_IDLE_CUTOFF = 10; // abaixo disso considera idle

// Pacote 5 bytes (40 bits)
byte pacote_drones[5];

// Rampa (filtro)
float motor1_speed = 0.0;
float motor2_speed = 0.0;
float motor3_speed = 0.0;
float motor4_speed = 0.0;

// Suavização da rampa 
float alpha = 0.98;

// ----------------- PROTÓTIPOS -----------------
void start(SoftwareSerial &port);
void pause(SoftwareSerial &port);
void blc_init(SoftwareSerial &port, int num_motor);
void blc_run_motor(SoftwareSerial &port);
void blc_reset(SoftwareSerial &port);
void blc_getVersion(SoftwareSerial &port);
void blc_startled(SoftwareSerial &port);
void blc_stopled(SoftwareSerial &port);
void blc_multicast(SoftwareSerial &port);
void blc_resetIRQ();

void gerar_pacote_motor(int m1, int m2, int m3, int m4, byte* packet);
void enviar_pacote(SoftwareSerial &port, const byte* packet);
void ramp(int m1_setpoint, int m2_setpoint, int m3_setpoint, int m4_setpoint);

// ----------------- SETUP -----------------
void setup() {
  //pinMode(4, INPUT);
  //pinMode(8, OUTPUT);
  //pinMode(POT_PIN, INPUT);
  //blc_resetIRQ();
  delay(1000);

  // Handshake completo (como o que funcionou)
  blc_init(mySerial,  1);
  blc_init(mySerial2, 2);
  blc_init(mySerial3, 3);
  blc_init(mySerial4, 4);
}

// ----------------- LOOP -----------------
void loop() {
  // Lê o potenciômetro
  int raw = analogRead(POT_PIN);

  // Idle abaixo do cutoff
  int setpoint;
  if (raw <= POT_IDLE_CUTOFF) {
    setpoint = 0; //
  } else {
    // Clampa a faixa útil e mapeia para 103..350 (faixa de operação do ESC)
    raw = constrain(raw, POT_MIN, POT_MAX);
    setpoint = map(raw, POT_MIN, POT_MAX, 0, 500);
  }

  // Usa o mesmo setpoint para os 4 motores (throttle geral)
  ramp(setpoint, setpoint, setpoint, setpoint);

  if (setpoint == 0) {
    gerar_pacote_motor(0, 0, 0, 0, pacote_drones);
    motor1_speed = 0.0;
    motor2_speed = 0.0;
    motor3_speed = 0.0;
    motor4_speed = 0.0;
  } else {
    // Pacote 40 bits
    gerar_pacote_motor((int)motor1_speed, (int)motor2_speed,
                      (int)motor3_speed, (int)motor4_speed, pacote_drones);
  }
  
  // Envia para os dois controladores
  enviar_pacote(mySerial,  pacote_drones);
  enviar_pacote(mySerial2, pacote_drones);
  enviar_pacote(mySerial3, pacote_drones);
  enviar_pacote(mySerial4, pacote_drones);

  // ~200 Hz
  delay(5);
}

// ----------------- SERIAL HELPERS -----------------
void start(SoftwareSerial &port)  { port.begin(115200); }
void pause(SoftwareSerial &port)  { port.end(); }

// ----------------- BLC / INIT -----------------
void blc_init(SoftwareSerial &port, int num_motor)
{
  start(port);  blc_reset(port);       pause(port);  delay(100);
  start(port);  blc_getVersion(port);  pause(port);  delay(100);
  start(port);  blc_run_motor(port);   pause(port);  delay(100);

  start(port);  port.write(num_motor); pause(port);  delay(100);
  start(port);  port.write(0x40);      pause(port);  delay(100);

  delay(1000);
  blc_startled(port);
  delay(500);
}

void blc_run_motor(SoftwareSerial &port)  { port.write(0xA1); }
void blc_reset(SoftwareSerial &port)      { port.write(0xE0); }
void blc_getVersion(SoftwareSerial &port) { port.write(0x91); }

void blc_startled(SoftwareSerial &port)
{
  start(port); port.write(0x60); pause(port); delay(100);
  start(port); port.write(0x1e); pause(port); delay(100);
}

void blc_stopled(SoftwareSerial &port)
{
  start(port); port.write(0x60);          pause(port); delay(100);
  start(port); port.write((uint8_t)0x00); pause(port); delay(100);
}

void blc_multicast(SoftwareSerial &port)
{
  for (int i = 1; i <= 6; i++) {
    start(port); port.write(0xa0); pause(port); delay(100);
  }
}

void blc_resetIRQ()
{
  if (digitalRead(4) == HIGH) {
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    delay(500);
    pinMode(8, INPUT);
  }
}

// ----------------- Rampa -----------------
void ramp(int m1_setpoint, int m2_setpoint, int m3_setpoint, int m4_setpoint)
{
  float m1_filt = alpha * motor1_speed + (1.0f - alpha) * m1_setpoint;
  float m2_filt = alpha * motor2_speed + (1.0f - alpha) * m2_setpoint;
  float m3_filt = alpha * motor3_speed + (1.0f - alpha) * m3_setpoint;
  float m4_filt = alpha * motor4_speed + (1.0f - alpha) * m4_setpoint;

  motor1_speed = m1_filt;
  motor2_speed = m2_filt;
  motor3_speed = m3_filt;
  motor4_speed = m4_filt;
}

// ----------------- Pacote 40 bits -----------------
void gerar_pacote_motor(int m1, int m2, int m3, int m4, byte* packet)
{
  // 9 bits por motor (0..511), faixa prática do ESC:
  //m1 = constrain(m1, 103, 350);
  //m2 = constrain(m2, 103, 350);
  //m3 = constrain(m3, 103, 350);
  //m4 = constrain(m4, 103, 350);

  packet[0] = 0b00100000 | (m1 >> 4);
  packet[1] = ((m1 & 0x0F) << 4) | (m2 >> 5);
  packet[2] = ((m2 & 0x1F) << 3) | (m3 >> 6);
  packet[3] = ((m3 & 0x3F) << 2) | (m4 >> 7);
  packet[4] = ((m4 & 0x7F) << 1); // bit final 0
}

// ----------------- Envio do pacote (contíguo) -----------------
void enviar_pacote(SoftwareSerial &port, const byte* packet)
{
  start(port);
  port.write(packet, 5);
  pause(port);
}
