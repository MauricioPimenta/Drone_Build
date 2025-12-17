/**
 * Teste de Unidade para a função gerar_pacote_motor.
 *
 * Este sketch NÃO usa SoftwareSerial nem controla motores.
 * Ele apenas chama a função 'gerar_pacote_motor' com 5 conjuntos
 * de dados de teste diferentes e imprime o pacote de 5 bytes resultante
 * no Serial Monitor do Arduíno.
 *
 * Como usar:
 * 1. Carregue este código no seu Arduíno.
 * 2. Abra o "Serial Monitor" (Ferramentas -> Serial Monitor).
 * 3. Configure a velocidade do monitor para "9600 baud".
 * 4. Observe os resultados impressos.
 */

void setup() {
  // Inicializa a comunicação Serial (para o PC, via USB)
  Serial.begin(9600);
  
  // Espera o Serial Monitor estar pronto (importante para Arduinos como o Leonardo,
  // mas não custa nada ter em outros)
  while (!Serial);
  
  delay(1000); // Uma pequena pausa para garantir
  
  Serial.println("--- INICIANDO TESTE DA FUNCAO gerar_pacote_motor ---");

  // Cria um array de 5 bytes para guardar o resultado
  byte meu_pacote[5];

  // --- Teste 1: Todos os motores em 0 ---
  Serial.println("\n--- Teste 1: (0, 0, 0, 0) ---");
  gerar_pacote_motor(0, 0, 0, 0, meu_pacote);
  printPacket(meu_pacote);

  // --- Teste 2: Todos os motores na velocidade MÁXIMA (511) ---
  Serial.println("\n--- Teste 2: (511, 511, 511, 511) ---");
  gerar_pacote_motor(511, 511, 511, 511, meu_pacote);
  printPacket(meu_pacote);

  // --- Teste 3: Os valores do seu exemplo original ---
  Serial.println("\n--- Teste 3: (300, 310, 320, 330) ---");
  gerar_pacote_motor(300, 310, 320, 330, meu_pacote);
  printPacket(meu_pacote);

  // --- Teste 4: Valores baixos e diferentes ---
  Serial.println("\n--- Teste 4: (1, 2, 3, 4) ---");
  gerar_pacote_motor(1, 2, 3, 4, meu_pacote);
  printPacket(meu_pacote);

  // --- Teste 5: Valores que testam as "fronteiras" dos bytes ---
  // (Ex: 15 = 000001111, 31 = 000011111, etc.)
  Serial.println("\n--- Teste 5: (15, 31, 63, 127) ---");
  gerar_pacote_motor(15, 31, 63, 127, meu_pacote);
  printPacket(meu_pacote);
  
  Serial.println("\n--- TESTE CONCLUIDO ---");
}

void loop() {
  // O loop fica vazio, pois o teste só precisa rodar uma vez.
}

// --- FUNÇÃO ATUALIZADA (Copiada do seu outro arquivo) ---
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
  m1 = constrain(m1, 0, 511);
  m2 = constrain(m2, 0, 511);
  m3 = constrain(m3, 0, 511);
  m4 = constrain(m4, 0, 511);

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


/**
 * Função auxiliar para imprimir o pacote de 5 bytes de forma legível.
 */
void printPacket(byte* packet) {
  // Imprime em Decimal
  Serial.print("  Bytes (DEC): [ ");
  for (int i = 0; i < 5; i++) {
    Serial.print(packet[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println(" ]");

  // Imprime em Hexadecimal (ótimo para depurar bits)
  Serial.print("  Bytes (HEX): [ ");
  for (int i = 0; i < 5; i++) {
    Serial.print("0x");
    if (packet[i] < 0x10) Serial.print("0"); // Adiciona zero à esquerda se for 0-F
    Serial.print(packet[i], HEX);
    if (i < 4) Serial.print(", ");
  }
  Serial.println(" ]");
}