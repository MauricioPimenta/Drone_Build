// Teensy 4.0: Potentiometer -> Serial Plotter (0..511)
const int POT_PIN = A0;                 // pin 14 on Teensy 4.0
const unsigned long BAUD = 115200;

void setup() {
  Serial.begin(BAUD);
  while (!Serial) { /* wait for USB serial */ }

  // Ensure 10-bit reads (0..1023) on Teensy 4.0
  analogReadResolution(10);
  // Optional: average multiple samples to reduce noise
  analogReadAveraging(8);
}

void loop() {
  int raw = analogRead(POT_PIN);                 // 0..1023
  int pot_0_511 = map(raw, 0, 1023, 0, 511);     // 0..511
  pot_0_511 = constrain(pot_0_511, 0, 511);

  // Serial Plotter format: labels and values on one line, separated by tabs
  Serial.print("raw:");         Serial.print(raw);
  Serial.print("\tpot_0_511:"); Serial.println(pot_0_511);

  delay(10); // ~100 Hz
}
