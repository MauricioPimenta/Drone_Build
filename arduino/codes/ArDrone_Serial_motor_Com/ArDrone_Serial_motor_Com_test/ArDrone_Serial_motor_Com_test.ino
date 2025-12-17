// UNO -> ESC (AR.Drone), teste de vida para ver no osciloscópio
// Ligações: pull-up 10k a +5V no DATA; TX0 ->|— DATA (ânodo no DATA, cátodo no TX), opcional 330R no TX.
// ESC: +5V no pino 2, ~12V no pino 1, GND comum no pino 5.

#include <Arduino.h>

const uint8_t CMD = 0x40;        // comando que costuma ter resposta
const unsigned GUARD_US = 60;    // pequeno intervalo após TX
const unsigned PERIOD_MS = 20;    // repete a cada 50 ms

void setup() {
  // UART 115200 8N1 na Serial0 (pinos 0/1) → ESC
  Serial.begin(115200);
  delay(500); // dá tempo do ESC iniciar lógica
}

void loop() {
  // 1) envia 1 byte que exige resposta (0x40)
  Serial.write(CMD);
  Serial.flush();
  delayMicroseconds(GUARD_US); // libera a linha pro ESC responder

  // 2) Espera um pouco antes de novo disparo (tempo largo p/ você ver no scope)
  delay(PERIOD_MS);
}