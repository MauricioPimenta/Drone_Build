#include <SoftwareSerial.h>

// --- Motor 1 ---
const int SOFT_RX_PIN_1 = 10;
const int SOFT_TX_PIN_1 = 11;
SoftwareSerial mySerial(SOFT_RX_PIN_1, SOFT_TX_PIN_1);

// --- Motor 2 (ESCOLHA PINOS NOVOS E LIVRES) ---
const int SOFT_RX_PIN_2 = 8;
const int SOFT_TX_PIN_2 = 9;
SoftwareSerial mySerial2(SOFT_RX_PIN_2, SOFT_TX_PIN_2);

void setup() {
  pinMode(4, INPUT);
  pinMode(8, OUTPUT);
  blc_resetIRQ();
  delay(1000);
  blc_init(mySerial, 1);
  blc_init(mySerial2, 2);
}
 
void loop() {
  run_motor(mySerial);
  run_motor(mySerial2);
  //delay(50);
}

// Esta função agora inicia a porta serial especificada
void start(SoftwareSerial &port)
{
  // AVISO: 115200 é MUITO RÁPIDO para duas SoftwareSerial.
  port.begin(115200);
}

void pause(SoftwareSerial &port)
{
  port.end();
}

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

  // Initialisation moteur
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
  port.write((uint8_t)0x00);
  pause(port);
  delay(100);
}
 
void blc_multicast(SoftwareSerial &port)
{
  for (int i = 1; i <= 6; i++) {
    start(port);
    port.write(0xa0);
    pause(port);
    delay(100);
  }
}
 
void run_motor(SoftwareSerial &port)
{
  start(port);
  port.write(40);
  pause(port);
  delay(5);

  start(port);
  port.write(39);
  pause(port);
  delay(5);

  start(port);
  port.write(251);
  pause(port);
  delay(5);

  start(port);
  port.write(253);
  pause(port);
  delay(5);

  start(port);
  port.write(254);
  pause(port);
  delay(5);
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