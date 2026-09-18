#include <Arduino.h>
#include <HX711.h>

// --- Definições de Pinos ---
const int HX711_DOUT = 4;
const int HX711_SCK = 5;

HX711 scale;
float calibration_factor = 113.0; // Valor identificado usando pesos conhecidos


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("--- Rotina de Calibração HX711 ---");
  Serial.println("1. Remova qualquer peso/tensão da célula de carga.");
  Serial.println("Iniciando o sensor em 3 segundos...");
  delay(3000);

  scale.begin(HX711_DOUT, HX711_SCK);
  
  // Reseta a escala para obter o valor bruto
  scale.set_scale();
  
  // Zera a balança (salva o offset)
  scale.tare(10); 
  
  Serial.println("Balança zerada!");
  Serial.println("2. Coloque o peso conhecido sobre a célula de carga.");
  Serial.println("3. Envie as letras pelo monitor serial para ajustar o fator de calibração:");
  Serial.println("   'a' (aumenta 1000)   |   'z' (diminui 1000)");
  Serial.println("   's' (aumenta 100)    |   'x' (diminui 100)");
  Serial.println("   'd' (aumenta 10)     |   'c' (diminui 10)");
  Serial.println("   'f' (aumenta 1)      |   'v' (diminui 1)");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Atualiza o fator de calibração
  scale.set_scale(calibration_factor);

  // Lê a força (média de 5 amostras) e imprime
  Serial.print("Leitura Atual: ");
  Serial.print(scale.get_units(5), 4); // 4 casas decimais
  Serial.print(" \t| Fator de Calibração: ");
  Serial.println(calibration_factor);

  // Verifica se há comandos digitados no Monitor Serial
  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == 'a') calibration_factor += 1000;
    else if (temp == 'z') calibration_factor -= 1000;
    else if (temp == 's') calibration_factor += 100;
    else if (temp == 'x') calibration_factor -= 100;
    else if (temp == 'd') calibration_factor += 10;
    else if (temp == 'c') calibration_factor -= 10;
    else if (temp == 'f') calibration_factor += 1;
    else if (temp == 'v') calibration_factor -= 1;
    else if (temp == 'o') scale.tare(10);
  }
  
  // Pequeno atraso para não inundar o monitor serial
  delay(250); 
}