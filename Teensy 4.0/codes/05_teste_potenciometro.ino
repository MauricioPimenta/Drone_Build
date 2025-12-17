// Potenciômetro -> Serial Plotter (0..511)
const int POT_PIN = A0;       // pino central do potenciômetro
const unsigned long BAUD = 115200;

void setup() {
  Serial.begin(BAUD);
}

void loop() {
  int raw = analogRead(POT_PIN); 
  int pot_0_511 = map(raw, 0, 1023, 0, 511);
  pot_0_511 = constrain(pot_0_511, 0, 511);

  // Formato compatível com o Serial Plotter (rótulos + valores na mesma linha)
  Serial.print("raw:");        Serial.print(raw);
  Serial.print("\tpot_0_511:");Serial.println(pot_0_511);

  delay(10); // ~100 Hz de atualização
}
