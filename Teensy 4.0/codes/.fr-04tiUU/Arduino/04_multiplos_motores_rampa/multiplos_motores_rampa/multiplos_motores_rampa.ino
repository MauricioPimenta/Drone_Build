#include <SoftwareSerial.h>

// --- Motor 1 (Drone 1) ---
const int SOFT_RX_PIN_1 = 10;
const int SOFT_TX_PIN_1 = 11;
SoftwareSerial mySerial(SOFT_RX_PIN_1, SOFT_TX_PIN_1);

// --- Motor 2 (Drone 2) ---
const int SOFT_RX_PIN_2 = 8;
const int SOFT_TX_PIN_2 = 9;
SoftwareSerial mySerial2(SOFT_RX_PIN_2, SOFT_TX_PIN_2);

// Arrays para armazenar os pacotes de 5 bytes
byte pacote_drones[5];

// Novas variáveis globais para a rampa (filtro)
float motor1_speed = 0.0;
float motor2_speed = 0.0;
float motor3_speed = 0.0;
float motor4_speed = 0.0;

float alpha = 0.99;

void setup() {
  delay(1000);
  //Serial.begin(115200);
  // Inicializa os dois motores (controladores)
  blc_init(mySerial, 1);
  blc_init(mySerial2, 2);
}

// --- FUNCAO DE RAMPA ---
void ramp(int m1_setpoint, int m2_setpoint, int m3_setpoint, int m4_setpoint){

  // ERRO CORRIGIDO: Devem ser 'float' para não perder precisão
  float m1_filt, m2_filt, m3_filt, m4_filt;
  
  // ERRO CORRIGIDO: Faltavam 8 pontos-e-vírgula (;) aqui
  m1_filt = alpha * motor1_speed + (1.0 - alpha) * m1_setpoint;
  m2_filt = alpha * motor2_speed + (1.0 - alpha) * m2_setpoint;
  m3_filt = alpha * motor3_speed + (1.0 - alpha) * m3_setpoint;
  m4_filt = alpha * motor4_speed + (1.0 - alpha) * m4_setpoint;

  motor1_speed = m1_filt;
  motor2_speed = m2_filt;
  motor3_speed = m3_filt;
  motor4_speed = m4_filt;
}

void loop() {
  // --- Drone 1 ---
  // Defina as velocidades desejadas (0-511 PARA TODOS)
  int m1 = 250; // Exemplo
  int m2 = 200; // Exemplo
  int m3 = 200; // Exemplo
  int m4 = 200; // Exemplo
  
  // Ajusta uma rampa de velocidade para o motor alcançar a velocidade desejada
  // ramp(m1, m2, m3, m4);

  // Gera o pacote de 5 bytes com base nas velocidades
  //gerar_pacote_motor((int)motor1_speed, (int)motor2_speed, (int)motor3_speed, (int)motor4_speed, pacote_drones);
  
  // IDLE
  gerar_pacote_motor(1, 1, 1, 1, pacote_drones);

  // Envia o mesmo pacote para os dois drones
  enviar_pacote(mySerial, pacote_drones);
  enviar_pacote(mySerial2, pacote_drones);

  //Serial.print(m1); // The target setpoint (a flat line)
  //Serial.print(" ");
  //Serial.println((int)motor1_speed); // The filtered speed (the curve)

  // Delay para definir a taxa de atualização (ex: 5ms = 200Hz)
  // Ajuste conforme necessário
  // delay(5);
}

// Esta função agora inicia a porta serial especificada
void start(SoftwareSerial &port)
{
  // AVISO: 115200 é MUITO RÁPIDO para duas SoftwareSerial.
  port.begin(115200);
}

// Esta função agora para a porta serial especificada
void pause(SoftwareSerial &port)
{
  port.end();
}

// --- FUNÇÃO DE PACOTE (VALIDADA) ---
// Gera o pacote de 5 bytes com base no protocolo CORRETO:
// 3 bits 'start' (001)
// 9 bits motor 1
// 9 bits motor 2
// 9 bits motor 3
// 9 bits motor 4
// 1 bit 'end' (0)
// TOTAL: 3 + 36 + 1 = 40 bits = 5 bytes
void gerar_pacote_motor(int m1, int m2, int m3, int m4, byte* packet) {
  
  // Garante que os valores de velocidade estejam dentro dos limites (9 bits = 0-511)
  //m1 = constrain(m1, 103, 350);
  //m2 = constrain(m2, 103, 350);
  //m3 = constrain(m3, 103, 350);
  //m4 = constrain(m4, 103, 350);

  // Empacota os bits de acordo com o protocolo de 40 bits
  
  // Byte 1: 001 + 5 bits mais altos do motor1 (m1[8:4])
  packet[0] = 0b00100000 | (m1 >> 4);

  // Byte 2: 4 bits mais baixos do motor1 (m1[3:0]) + 4 bits mais altos do motor2 (m2[8:5])
  packet[1] = ((m1 & 0x0F) << 4) | (m2 >> 5);

  // Byte 3: 5 bits mais baixos do motor2 (m2[4:0]) + 3 bits mais altos do motor3 (m3[8:6])
  packet[2] = ((m2 & 0x1F) << 3) | (m3 >> 6);

  // Byte 4: 6 bits mais baixos do motor3 (m3[5:0]) + 2 bits mais altos do motor4 (m4[8:7])
  packet[3] = ((m3 & 0x3F) << 2) | (m4 >> 7);

  // Byte 5: 7 bits mais baixos do motor4 (m4[6:0]) + bit final 0
  packet[4] = ((m4 & 0x7F) << 1); // O shift para a esquerda já insere o 0 no final
}

// --- FUNÇÃO DE ENVIO ---
// Envia um pacote de 5 bytes para a porta serial especificada.
void enviar_pacote(SoftwareSerial &port, byte* packet) {
  start(port);
  port.write(packet[0]);
  port.write(packet[1]);
  port.write(packet[2]);
  port.write(packet[3]);
  port.write(packet[4]);
  pause(port);

  //start(port);
  ////port.write(packet[0]);
  //pause(port);
  //delay(5);

  //start(port);
  //port.write(packet[1]);
  //pause(port);
  //delay(5);

  //start(port);
  //port.write(packet[2]);
  //pause(port);
  //delay(5);

  //start(port);
  //port.write(packet[3]);
  //pause(port);
  //delay(5);

  //start(port);
  //port.write(packet[4]);
  //pause(port);
  //delay(5);
  // O delay de 5ms foi movido para o loop principal
}

// --- Funções BLC (Refatoradas para aceitar a porta) ---
void blc_init(SoftwareSerial &port, int num_motor)
{
  // Initialisation multicast
  start(port);
  blc_reset(port);
  pause(port);
  delay(100);

  start(port);
  blc_getVersion(port);
  pause(port);
  delay(100);

  start(port);
  blc_run_motor(port);
  pause(port);
  delay(100);

  start(port);
  port.write(num_motor);
  pause(port);
  delay(100);

  start(port);
  port.write(0x40);
  pause(port);
  delay(100);

  delay(1000);

  blc_startled(port);

  delay(1000);
}

void blc_run_motor(SoftwareSerial &port)
{
  port.write(0xA1);
}

void blc_reset(SoftwareSerial &port)
{
  port.write(0xE0);
}

void blc_getVersion(SoftwareSerial &port)
{
  port.write(0x91);
}

void blc_startled(SoftwareSerial &port)
{
  start(port);
  port.write(0x60);
  pause(port);
  delay(100);

  start(port);
  port.write(0x1e);
  pause(port);
  delay(100);
}

void blc_stopled(SoftwareSerial &port)
{
  start(port);
  port.write(0x60);
  pause(port);
  delay(100);

  start(port);
  // Cast para (uint8_t) para evitar ambiguidade com 0x00
  port.write((uint8_t)0x00);
  pause(port);
  delay(100);
}

void blc_multicast(SoftwareSerial &port)
{
  // Esta função não está sendo usada, mas está aqui caso precise
  for (int i = 1; i <= 6; i++) {
    start(port);
    port.write(0xa0);
    pause(port);
    delay(100);
  }
}

// Esta função parece ser global e não específica de uma porta serial
void blc_resetIRQ()
{
  if (digitalRead(4) == HIGH) {
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    delay(500);
    pinMode(8, INPUT);
  }
}