/* AR.Drone 1.0 - 1 motor no Arduino Uno (TX-only)
 * Ligações:
 *   - Motor Pin1 (vermelho)  -> +11~12.6 V (fonte/bateria)
 *   - Motor Pin2 (Branco)    -> +5 V (regulado)
 *   - Motor Pin3 (Verde)   -> Arduino D1 (TX) via resistor 330 Ω
 *   - Motor Pin4 (Amarelo/Rosa)      -> (opcional) não ligado - IRQ
 *   - Motor Pin5 (preto)     -> GND comum (Arduino e fonte)
 *
 *
 * Passos: E0 (status), ID=0x01, A0 x6, e frames de velocidade ~200 Hz.
 * OBS: Após upload, NÃO deixe o USB ligado; alimente por fonte externa.
 */

#include <Arduino.h>
// #include <SingleWireSerial.h>

// ===== Config =====
static const uint16_t SPEED = 220;  // 0..511 (9 bits). Comece ~150..250
static const uint32_t FRAME_PERIOD_US = 5000; // 5 ms (200 Hz)

// ===== Helpers de envio =====
static inline void sendByte(uint8_t b) {
  // Envia 1 byte cru na UART
  UDR0 = b;                 // coloca no registrador do UART
  while (!(UCSR0A & _BV(TXC0))) { /* espera terminar */ }
  UCSR0A |= _BV(TXC0);      // limpa flag de TX completo
}

static void sendBytes(uint8_t *buf, size_t n) {
  
  // buf[0] = 0b00101101;
  // buf[1] = 0b11000000;
  // buf[2] = 0b00000000;
  // buf[3] = 0b00000000;
  // buf[4] = 0b00000000;

  // Serial.println();
  // Serial.print("Sent: ");
  // Serial.print(buf_data, HEX);
  // Serial.print(" -- ");
  // Serial.println(buf_data, BIN);

  // uint64_t buf_data = 0;

  // Serial.println();
  // Serial.print("Sent: ");

  // for (size_t i = 0; i < n; i++) {
  //   Serial.print(buf[i], BIN);
  //   Serial.print(" ");

  //   buf_data = (buf_data << 8) | buf[i];
  // }
  // Serial.print(" -- ");
  // for (int i = 4; i >= 0; i--) {
  //   if (buf[i] < 0x10) Serial.print('0');
  //   Serial.print(buf[i], HEX);
  // }
  // Serial.print(" -- ");
  // // Print in binary (as text)
  // for (int i = 0; i <= 4; i++) {
  //   for (int b = 7; b >= 0; b--) {
  //     Serial.print((buf[i] >> b) & 1);
  //   }
  // }
  // Serial.println();

  for (size_t i = 0; i < n; ++i) {
    // Serial.print("buffer["); Serial.print(i); Serial.print("] = ");
    Serial.write(buf[i]);
    Serial.flush(); // garante saída antes do próximo
  }

}

// ===== Protocolo mínimo =====

// Atribui ID do ESC (1..4). Para 1 motor, use 0x01
static void assignMotorID(uint8_t id) {
  // No protocolo reverso, enviar apenas o byte do ID já funciona
  sendByte(id); // 0x01 para motor #1
  delay(10);
}

// Entra no modo "multicast" mandando 0xA0 seis vezes
static void enterMulticastMode() {
  for (int i = 0; i < 6; ++i) {
    sendByte(0xA0);
    delay(2);
  }
}

/* Monta e envia o quadro de velocidades (para 4 motores) a 200 Hz.
 * Cada motor usa 9 bits (0..511). O pacote total tem 5 bytes.
 *
 * ATENÇÃO: O empacotamento de 4x9 bits em 5 bytes abaixo segue um
 * mapeamento usado pela comunidade (36 bits = 0..35). Em alguns ESCs
 * pode ser necessário ajustar a ordem/bitshift. Teste com valores simples:
 *   - tudo zero (parado), depois só M1 > 0, os demais zero.
 */
static void sendSpeedFrame(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  uint16_t pwm1 = m1 & 0x1FF;
  uint16_t pwm2 = m2 & 0x1FF; 
  uint16_t pwm3 = m3 & 0x1FF;
  uint16_t pwm4 = m4 & 0x1FF;

  // Concatena os 4 campos de 9 bits: [m1(35..27) | m2(26..18) | m3(17..9) | m4(8..0)]
  // Vamos construir um buffer de 5 bytes (MSB -> LSB)
  uint8_t b[5];

  b[0] = 0x20 | ((pwm1 & 0x1ff) >> 4);
	b[1] = ((pwm1 & 0x1ff) << 4) | ((pwm2 & 0x1ff) >> 5);
	b[2] = ((pwm2 & 0x1ff) << 3) | ((pwm3 & 0x1ff) >> 6);
	b[3] = ((pwm3 & 0x1ff) << 2) | ((pwm4 & 0x1ff) >> 7);
	b[4] = ((pwm4 & 0x1ff) << 1);
  

  sendBytes(b, 5);
}

void setup() {
  // Configura UART hardware do UNO em 115200 8N1
  Serial.end();                // garante reset da UART
  Serial.begin(115200);
  delay(500);                  // espera ESC bootar a lógica

  // 1) Opcional: pedir status (não vamos ler resposta neste primeiro teste)
  sendByte(0xE0);
  delay(10);

  // 2) Atribui ID #1
  assignMotorID(0x01);

  // 3) Entra em multicast
  enterMulticastMode();
}

void loop() {
  static uint32_t t0 = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - t0) >= FRAME_PERIOD_US) {
    t0 += FRAME_PERIOD_US;

    // Envia frame de velocidade a cada 5 ms.
    // Controlamos apenas M1, deixamos M2..M4 = 0.
    sendSpeedFrame(SPEED, 0, 0, 0);
  }
}