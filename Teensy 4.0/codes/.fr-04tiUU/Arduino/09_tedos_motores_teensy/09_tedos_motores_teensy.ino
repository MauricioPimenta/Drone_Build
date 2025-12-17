// ----------------- USER PINS / PORT SELECTION -----------------
// Choose which Teensy UARTs you will use (TX pins shown for reference):
//   Serial1 -> TX1 (pin 1)
//   Serial2 -> TX2 (pin 8)
//   Serial3 -> TX3 (pin 14)
//   Serial4 -> TX4 (pin 17)
// Adjust if you prefer other serial ports.

HardwareSerial* const ports[4] = { &Serial1, &Serial2, &Serial3, &Serial4 };

// ----------------- POTENTIOMETER -----------------
const int POT_PIN = A1;
const int POT_MIN = 20;    // ignore noise below this
const int POT_MAX = 1023;  // 10-bit ADC default on Teensy 4.0
const int POT_IDLE_CUTOFF = 40;

// ----------------- STATE -----------------
byte pacote_drones[5];

float motor1_speed = 0.0f;
float motor2_speed = 0.0f;
float motor3_speed = 0.0f;
float motor4_speed = 0.0f;

float alpha = 0.98f;

const uint32_t LOOP_PERIOD_MICROS = 5000;  // 200Hz

// ----------------- PROTOTYPES -----------------
void blc_init(HardwareSerial &port, int num_motor);
void blc_run_motor(HardwareSerial &port);
void blc_reset(HardwareSerial &port);
void blc_getVersion(HardwareSerial &port);
void blc_startled(HardwareSerial &port);
void blc_stopled(HardwareSerial &port);
void blc_multicast(HardwareSerial &port);
void blc_resetIRQ(); // left here if you later wire IRQ, not used now

void gerar_pacote_motor(int m1, int m2, int m3, int m4, byte* packet);
void enviar_pacote(HardwareSerial &port, const byte* packet);
void ramp(int m1_setpoint, int m2_setpoint, int m3_setpoint, int m4_setpoint);

// ----------------- SETUP -----------------
void setup() {
  // Serial monitor (optional):
  // Serial.begin(115200);

  // Init the 4 UARTs at 115200 (8N1)
  for (int i = 0; i < 4; ++i) {
    ports[i]->begin(115200);
  }

  delay(1000);

  // Full handshake per motor (same sequence you used)
  blc_init(*ports[0], 1);
  blc_init(*ports[1], 2);
  blc_init(*ports[2], 3);
  blc_init(*ports[3], 4);
}

// ----------------- LOOP -----------------
void loop() {
  uint32_t loop_start_micros = micros();

  int raw = analogRead(POT_PIN);

  int setpoint;
  if (raw <= POT_IDLE_CUTOFF) {
    setpoint = 0;
  } else {
    raw = constrain(raw, POT_MIN, POT_MAX);
    setpoint = map(raw, POT_MIN, POT_MAX, 0, 500);
  }

  ramp(setpoint, setpoint, setpoint, setpoint);

  if (setpoint == 0) {
    gerar_pacote_motor(0, 0, 0, 0, pacote_drones);
    motor1_speed = motor2_speed = motor3_speed = motor4_speed = 0.0f;
  } else {
    gerar_pacote_motor((int)motor1_speed, (int)motor2_speed,
                       (int)motor3_speed, (int)motor4_speed, pacote_drones);
  }

  // Send the same packet to all 4 ESCs
  for (int i = 0; i < 4; ++i) {
    enviar_pacote(*ports[i], pacote_drones);
  }

  uint32_t execution_time_micros = micros() - loop_start_micros;

  if (execution_time_micros < LOOP_PERIOD_MICROS) {
    // We do! Wait for the remaining time.
    uint32_t wait_time_micros = LOOP_PERIOD_MICROS - execution_time_micros;
    delayMicroseconds(wait_time_micros);
  }
}

// ----------------- BLC / INIT -----------------
void blc_init(HardwareSerial &port, int num_motor)
{
  blc_reset(port);       delay(100);
  blc_getVersion(port);  delay(100);
  blc_run_motor(port);   delay(100);

  port.write((uint8_t)num_motor); delay(100);
  port.write((uint8_t)0x40);      delay(100);

  delay(1000);
  blc_startled(port);
  delay(500);
}

void blc_run_motor(HardwareSerial &port)  { port.write((uint8_t)0xA1); }
void blc_reset(HardwareSerial &port)      { port.write((uint8_t)0xE0); }
void blc_getVersion(HardwareSerial &port) { port.write((uint8_t)0x91); }

void blc_startled(HardwareSerial &port)
{
  port.write((uint8_t)0x60); delay(100);
  port.write((uint8_t)0x1e); delay(100);
}

void blc_stopled(HardwareSerial &port)
{
  port.write((uint8_t)0x60);          delay(100);
  port.write((uint8_t)0x00);          delay(100);
}

void blc_multicast(HardwareSerial &port)
{
  for (int i = 1; i <= 6; i++) {
    port.write((uint8_t)0xa0); delay(100);
  }
}

// Kept for compatibility if you later wire an IRQ pin:
void blc_resetIRQ()
{
  // No-op for now on Teensy (implement if you wire IRQ lines)
}

// ----------------- RAMP -----------------
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

// ----------------- PACKET (40 bits) -----------------
void gerar_pacote_motor(int m1, int m2, int m3, int m4, byte* packet)
{
  // Keep full 0..511 range; clamp if/when needed
  // m1 = constrain(m1, 103, 350);
  // m2 = constrain(m2, 103, 350);
  // m3 = constrain(m3, 103, 350);
  // m4 = constrain(m4, 103, 350);

  packet[0] = 0b00100000 | (m1 >> 4);
  packet[1] = ((m1 & 0x0F) << 4) | (m2 >> 5);
  packet[2] = ((m2 & 0x1F) << 3) | (m3 >> 6);
  packet[3] = ((m3 & 0x3F) << 2) | (m4 >> 7);
  packet[4] = ((m4 & 0x7F) << 1); // final bit 0
}

// ----------------- SEND -----------------
void enviar_pacote(HardwareSerial &port, const byte* packet)
{
  port.write(packet, 5);
}
