/*
 * ============================================================================
 * UFES - LAB-AIR | Dissertação de Mestrado - Maurício Bittencourt Pimenta
 * Controladora de Voo para Quadrimotor (Teensy 4.1)
 * 
 * Modos implementados:
 *   0 = MODE_RATE      (Controle PID de Taxas Angulares)
 *   1 = MODE_STABILIZE (Controle PID em Cascata de Atitude: Ângulo -> Taxa)
 *   2 = MODE_ALT_HOLD  (Controle de Atitude + Altitude/Velocidade Vertical - Hover)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <PulsePosition.h>

// ============================================================================
// 1. CONFIGURAÇÕES DE HARDWARE E PINAGEM (PCB UFES - PARROT BEBop FRAME)
// ============================================================================
// Pinagem soldada na PCB dos Motores:
const int PIN_MTD = 3; // Motor Traseiro Direito (Back-Right) - CCW
const int PIN_MTE = 4; // Motor Traseiro Esquerdo (Back-Left) - CW
const int PIN_MDE = 5; // Motor Dianteiro Esquerdo (Front-Left) - CCW
const int PIN_MDD = 6; // Motor Dianteiro Direito (Front-Right) - CW

const int PIN_PPM_RX   = 14; // Entrada Receptor Rádio PPM (se utilizado)
const int PIN_BAT_VOLT = 15; // Divisor de tensão da bateria
const int PIN_LED_BUILTIN = 13;

// Configuração do circuito inversor da PCB (Transistor NPN 3.3V -> 5V)
// true  = Lógica invertida na PCB (Teensy LOW -> ESC HIGH)
// false = Lógica direta (Teensy HIGH -> ESC HIGH)
const bool INVERT_ESC_LOGIC = true;

// Frequência PWM dos ESCs:
// Use 50 Hz para os ESCs amarelos 30A atuais ou 250 Hz para ESCs rápidos
const int ESC_PWM_FREQ = 50; 
const float ESC_PERIOD_US = 1000000.0f / ESC_PWM_FREQ;
const int PWM_RES_BITS = 12;
const float PWM_MAX_DUTY = 4096.0f;

// Limiares de pulso dos ESCs (em microssegundos)
const int THROTTLE_CUTOFF = 1000; // Motores desligados
const int THROTTLE_IDLE   = 1180; // Rotação mínima em voo (evita estol dos motores)
const int THROTTLE_MAX    = 1800; // Limite do comando base (reserva 200us para atitude)
const int MOTOR_SAT_MAX   = 1999; // Saturação máxima enviada ao ESC
const int THROTTLE_HOVER  = 1500; // Pulso nominal de equilíbrio para voo pairado

// ============================================================================
// 2. DEFINIÇÃO DOS MODOS DE VOO E TEMPORIZAÇÃO DA MALHA (250 Hz)
// ============================================================================
enum FlightMode {
  MODE_RATE = 0,
  MODE_STABILIZE = 1,
  MODE_ALT_HOLD = 2
};

FlightMode currentMode = MODE_STABILIZE; // Modo inicial padrão
bool motorsArmed = false;

const float DT = 0.004f;                 // Período de amostragem = 4 ms (250 Hz)
const uint32_t LOOP_PERIOD_US = 4000;
uint32_t loopTimer = 0;

// ============================================================================
// 3. VARIÁVEIS DE SENSORES (IMU MPU-6050/6500/9250 E BARÔMETRO BMP280)
// ============================================================================
// Giroscópio e Acelerômetro
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;

float AccX, AccY, AccZ;
// Offsets de calibração do acelerômetro (ajustar conforme calibração estática)
float AccErrorX = 0.05f;
float AccErrorY = -0.01f;
float AccErrorZ = 0.11f;

float AngleRoll, AnglePitch;
float AccZInertial = 0;

// Filtro de Kalman 1D (Atitude: Roll e Pitch)
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4.0f;   // 2^2
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4.0f; // 2^2
float Kalman1DOutput[2] = {0, 0};

// Barômetro BMP280 (Parâmetros de compensação de fábrica)
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer = 0, AltitudeBarometerStartUp = 0;
bool barometerAvailable = false;

// Filtro de Kalman 2D (Altitude [cm] e Velocidade Vertical [cm/s])
float AltitudeKalman = 0, VelocityVerticalKalman = 0;
float P2D[2][2] = {{0, 0}, {0, 0}}; // Matriz de covariância 2x2 do Kalman 2D

// ============================================================================
// 4. VARIÁVEIS DE COMANDO E CONTROLADORES PID
// ============================================================================
PulsePositionInput ReceiverInput(RISING);
// Canais: [0]=Roll, [1]=Pitch, [2]=Throttle, [3]=Yaw, [4]=Arm/Cut, [5]=Mode
float ReceiverValue[8] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500};
bool useRadioPPM = false; // Se false, aceita comandos via Serial / Wi-Fi (ESP-07)

// Setpoints Desejados
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;
float DesiredAngleRoll = 0, DesiredAnglePitch = 0;
float DesiredAltitude = 50.0f;        // Setpoint de altura para hover [cm]
float DesiredVelocityVertical = 0.0f; // Setpoint de velocidade vertical [cm/s]
bool holdAltitudeTargetLocked = false;

// Erros, Memória PID e Saídas
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float ErrorVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical;

float InputRoll, InputPitch, InputYaw, InputThrottle = 1000;
float PIDReturn[3] = {0, 0, 0};

// --- Ganhos PID: Malha Interna de Taxas Angulares (Rate Loop) ---
float PRateRoll = 0.6f,  IRateRoll = 3.5f,  DRateRoll = 0.03f;
float PRatePitch = 0.6f, IRatePitch = 3.5f, DRatePitch = 0.03f;
float PRateYaw = 2.0f,   IRateYaw = 12.0f,  DRateYaw = 0.0f;

// --- Ganhos PID: Malha Externa de Ângulos de Atitude (Angle Loop) ---
float PAngleRoll = 2.0f,  IAngleRoll = 0.0f,  DAngleRoll = 0.0f;
float PAnglePitch = 2.0f, IAnglePitch = 0.0f, DAnglePitch = 0.0f;

// --- Ganhos PID: Malha de Altitude / Velocidade Vertical (Hover Loop) ---
float PPosZ = 1.2f; // Ganho Proporcional da malha externa de Posição Z (Altitude -> VelZ)
float PVelocityVertical = 3.5f, IVelocityVertical = 0.0015f, DVelocityVertical = 0.01f;

// Comandos finais em microssegundos para os 4 motores
float MotorInputMDE, MotorInputMDD, MotorInputMTD, MotorInputMTE;

// Buffer para Comandos Seriais / Wi-Fi ESP-07
const int SERIAL_BUF_SIZE = 64;
char serialBuf[SERIAL_BUF_SIZE];
int serialBufIdx = 0;

// ============================================================================
// 5. FUNÇÕES DE ACIONAMENTO PWM (COM COMPENSAÇÃO DE INVERSÃO LÓGICA)
// ============================================================================
void writeEscMicros(int pin, float pulseUs) {
  pulseUs = constrain(pulseUs, 1000.0f, 2000.0f);
  uint32_t duty;

  if (INVERT_ESC_LOGIC) {
    // PCB com transistor NPN inversor: Teensy LOW = ESC HIGH
    duty = (uint32_t)(PWM_MAX_DUTY - ((PWM_MAX_DUTY / ESC_PERIOD_US) * pulseUs));
  } else {
    // Ligação direta: Teensy HIGH = ESC HIGH
    duty = (uint32_t)((PWM_MAX_DUTY / ESC_PERIOD_US) * pulseUs);
  }
  analogWrite(pin, duty);
}

void stopAllMotors() {
  writeEscMicros(PIN_MDE, THROTTLE_CUTOFF);
  writeEscMicros(PIN_MDD, THROTTLE_CUTOFF);
  writeEscMicros(PIN_MTD, THROTTLE_CUTOFF);
  writeEscMicros(PIN_MTE, THROTTLE_CUTOFF);
}

// ============================================================================
// 6. FUNÇÕES DE LEITURA DE SENSORES (IMU E BARÔMETRO)
// ============================================================================
void readIMUSignals() {
  // 1. Configura Filtro Passa-Baixa Digital (DLPF = 10 Hz -> 0x05)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // 2. Configura Fundo de Escala do Acelerômetro (+/- 8g -> 4096 LSB/g -> 0x10)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // 3. Lê Registradores do Acelerômetro (0x3B a 0x40)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // 4. Configura Fundo de Escala do Giroscópio (+/- 500 deg/s -> 65.5 LSB/deg/s -> 0x08)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // 5. Lê Registradores do Giroscópio (0x43 a 0x48)
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Conversão para unidades físicas
  RateRoll  = (float)GyroX / 65.5f;
  RatePitch = (float)GyroY / 65.5f;
  RateYaw   = (float)GyroZ / 65.5f;

  AccX = ((float)AccXLSB / 4096.0f) - AccErrorX;
  AccY = ((float)AccYLSB / 4096.0f) - AccErrorY;
  AccZ = ((float)AccZLSB / 4096.0f) - AccErrorZ;

  // Cálculo trigonométrico dos ângulos absolutos pelo acelerômetro [graus]
  AngleRoll  =  atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0f / PI);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0f / PI);
}

void initBarometerBMP280() {
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57); // Indoor navigation: osrs_t=x2, osrs_p=x16, Normal Mode
  if (Wire.endTransmission() != 0) {
    barometerAvailable = false;
    return;
  }
  barometerAvailable = true;

  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(0x14); // Filtro IIR = 16, t_sb = 0.5ms
  Wire.endTransmission();

  // Leitura dos 24 bytes de calibração (0x88 a 0x9F)
  uint8_t data[24];
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24);
  for (int i = 0; i < 24 && Wire.available(); i++) {
    data[i] = Wire.read();
  }

  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);
}

void readBarometerSignals() {
  if (!barometerAvailable) return;

  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6);
  if (Wire.available() < 6) return;

  uint32_t press_msb  = Wire.read();
  uint32_t press_lsb  = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb   = Wire.read();
  uint32_t temp_lsb   = Wire.read();
  uint32_t temp_xlsb  = Wire.read();

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12)  | (temp_lsb << 4)  | (temp_xlsb >> 4);

  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0) {
    p = 0;
  } else {
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
      p = (p << 1) / ((unsigned long int)var1);
    } else {
      p = (p / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
  }

  double pressure = (double)p / 100.0;
  // Fórmula barométrica convertida para centímetros [cm]
  AltitudeBarometer = 44330.0f * (1.0f - pow(pressure / 1013.25, 1.0 / 5.255)) * 100.0f;
}

// ============================================================================
// 7. FILTROS DE KALMAN 1D (ATITUDE) E 2D (ALTITUDE / VELOCIDADE VERTICAL)
// ============================================================================
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // 1. Predição com taxa do giroscópio
  KalmanState = KalmanState + DT * KalmanInput;
  // Variância do processo: (DT^2) * (4 deg/s)^2
  KalmanUncertainty = KalmanUncertainty + (DT * DT * 16.0f);

  // 2. Atualização com ângulo do acelerômetro (incerteza de 3 deg -> variância = 9)
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9.0f);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1.0f - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void kalman_2d(float accZ_cms2, float baroAlt_cm) {
  // Expansão escalar exata do Filtro de Kalman 2D (Estados: S = [Altitude; VelVertical])
  // Desvio padrão do acelerômetro sigma_a = 10 cm/s^2 -> var_a = 100
  // Desvio padrão do barômetro sigma_b = 30 cm -> R = 900
  const float var_a = 100.0f;
  const float R_baro = 900.0f;
  const float dt2 = DT * DT;
  const float dt3 = dt2 * DT;
  const float dt4 = dt2 * dt2;

  // 1. Predição do vetor de estados: S(k) = F*S(k-1) + G*AccZ
  AltitudeKalman = AltitudeKalman + DT * VelocityVerticalKalman + 0.5f * dt2 * accZ_cms2;
  VelocityVerticalKalman = VelocityVerticalKalman + DT * accZ_cms2;

  // 2. Predição da matriz de covariância: P(k) = F*P(k-1)*F^T + Q
  float p00 = P2D[0][0] + DT * (P2D[1][0] + P2D[0][1]) + dt2 * P2D[1][1] + 0.25f * dt4 * var_a;
  float p01 = P2D[0][1] + DT * P2D[1][1] + 0.5f * dt3 * var_a;
  float p10 = P2D[1][0] + DT * P2D[1][1] + 0.5f * dt3 * var_a;
  float p11 = P2D[1][1] + dt2 * var_a;

  // 3. Ganho de Kalman: L = H*P*H^T + R = p00 + R_baro; K = P*H^T / L
  float L = p00 + R_baro;
  float K0 = p00 / L;
  float K1 = p10 / L;

  // 4. Atualização do estado com a medição do barômetro: S = S + K*(M - H*S)
  float innovation = baroAlt_cm - AltitudeKalman;
  AltitudeKalman = AltitudeKalman + K0 * innovation;
  VelocityVerticalKalman = VelocityVerticalKalman + K1 * innovation;

  // 5. Atualização da covariância: P = (I - K*H)*P
  P2D[0][0] = (1.0f - K0) * p00;
  P2D[0][1] = (1.0f - K0) * p01;
  P2D[1][0] = p10 - K1 * p00;
  P2D[1][1] = p11 - K1 * p01;
}

// ============================================================================
// 8. EQUAÇÃO DO CONTROLADOR PID DISCRETO E RESET ANTI-WINDUP
// ============================================================================
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * DT / 2.0f;
  Iterm = constrain(Iterm, -400.0f, 400.0f); // Anti-windup do termo integral

  float Dterm = D * (Error - PrevError) / DT;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -400.0f, 400.0f); // Saturação da saída PID

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid() {
  PrevErrorRateRoll = 0;  PrevErrorRatePitch = 0;  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;  PrevItermRatePitch = 0;  PrevItermRateYaw = 0;
  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
  PrevErrorVelocityVertical = 0;
  PrevItermVelocityVertical = 0;
}

// ============================================================================
// 9. LEITURA DE COMANDOS (RÁDIO PPM OU SERIAL/WI-FI ESP-07)
// ============================================================================
void readReceiverPPM() {
  int chNum = ReceiverInput.available();
  if (chNum > 0) {
    useRadioPPM = true;
    for (int i = 1; i <= chNum && i <= 8; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
    // Canal 5: Chave de Armamento / Corte de segurança (Acima de 1500 = Armado)
    if (chNum >= 5) {
      motorsArmed = (ReceiverValue[4] > 1500);
    }
  }
}

void processSerialCommand(char *cmd) {
  while (*cmd == ' ') cmd++;
  // Comandos rápidos via Serial / Wi-Fi ESP-07:
  // "ARM"      -> Arma os motores (se throttle estiver baixo)
  // "DISARM"   -> Corta os motores imediatamente (Emergência)
  // "MODE <n>" -> 0=RATE, 1=STABILIZE, 2=ALT_HOLD
  // "T <us>"   -> Define Throttle [1000..2000]
  // "R <us>"   -> Define Roll [1000..2000] (1500=centro)
  // "P <us>"   -> Define Pitch [1000..2000] (1500=centro)
  // "Y <us>"   -> Define Yaw [1000..2000] (1500=centro)
  // "Z <cm>"   -> Define altura alvo para Hover em cm (ex: Z 80)
  // "STATUS"   -> Imprime telemetria atual
  if (strcasecmp(cmd, "DISARM") == 0 || strcasecmp(cmd, "STOP") == 0) {
    motorsArmed = false;
    ReceiverValue[2] = 1000;
    stopAllMotors();
    Serial.println("[SAFETY] Motores DESARMADOS.");
  } 
  else if (strcasecmp(cmd, "ARM") == 0) {
    if (ReceiverValue[2] < 1050) {
      motorsArmed = true;
      Serial.println("[ARMED] Motores ARMADOS.");
    } else {
      Serial.println("[ERRO] Reduza o Throttle (<1050) antes de armar!");
    }
  } 
  else if (strncasecmp(cmd, "MODE ", 5) == 0) {
    int m = atoi(cmd + 5);
    if (m >= 0 && m <= 2) {
      currentMode = (FlightMode)m;
      holdAltitudeTargetLocked = false;
      Serial.print("[MODO] Alterado para: ");
      Serial.println(m == 0 ? "RATE" : (m == 1 ? "STABILIZE" : "ALT_HOLD"));
    }
  } 
  else if (strncasecmp(cmd, "T ", 2) == 0) {
    ReceiverValue[2] = constrain(atof(cmd + 2), 1000.0f, 2000.0f);
  } 
  else if (strncasecmp(cmd, "R ", 2) == 0) {
    ReceiverValue[0] = constrain(atof(cmd + 2), 1000.0f, 2000.0f);
  } 
  else if (strncasecmp(cmd, "P ", 2) == 0) {
    ReceiverValue[1] = constrain(atof(cmd + 2), 1000.0f, 2000.0f);
  } 
  else if (strncasecmp(cmd, "Y ", 2) == 0) {
    ReceiverValue[3] = constrain(atof(cmd + 2), 1000.0f, 2000.0f);
  } 
  else if (strncasecmp(cmd, "Z ", 2) == 0) {
    DesiredAltitude = constrain(atof(cmd + 2), 0.0f, 300.0f);
    holdAltitudeTargetLocked = true;
    Serial.print("[HOVER] Altitude alvo definida: ");
    Serial.print(DesiredAltitude);
    Serial.println(" cm");
  } 
  else if (strcasecmp(cmd, "STATUS") == 0) {
    Serial.print("Mode:"); Serial.print((int)currentMode);
    Serial.print(" | Roll:"); Serial.print(KalmanAngleRoll, 1);
    Serial.print(" | Pitch:"); Serial.print(KalmanAnglePitch, 1);
    Serial.print(" | Alt(cm):"); Serial.print(AltitudeKalman, 1);
    Serial.print(" | VelZ(cm/s):"); Serial.print(VelocityVerticalKalman, 1);
    Serial.print(" | MDE:"); Serial.print(MotorInputMDE, 0);
    Serial.print(" MDD:"); Serial.print(MotorInputMDD, 0);
    Serial.print(" MTD:"); Serial.print(MotorInputMTD, 0);
    Serial.print(" MTE:"); Serial.println(MotorInputMTE, 0);
  }
}

void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuf[serialBufIdx] = '\0';
      if (serialBufIdx > 0) processSerialCommand(serialBuf);
      serialBufIdx = 0;
    } else if (serialBufIdx < SERIAL_BUF_SIZE - 1) {
      serialBuf[serialBufIdx++] = c;
    } else {
      serialBufIdx = 0;
    }
  }
}

// ============================================================================
// 10. SETUP INICIAL E CALIBRAÇÃO DE SENSORES
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_BUILTIN, HIGH);

  // 1. Inicializa PWM de Hardware dos 4 Motores e garante corte inicial
  analogWriteFrequency(PIN_MTD, ESC_PWM_FREQ);
  analogWriteFrequency(PIN_MTE, ESC_PWM_FREQ);
  analogWriteFrequency(PIN_MDE, ESC_PWM_FREQ);
  analogWriteFrequency(PIN_MDD, ESC_PWM_FREQ);
  analogWriteResolution(PWM_RES_BITS);
  stopAllMotors();

  // 2. Inicializa Barramento I2C a 400 kHz e acorda a IMU (0x68)
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // 3. Inicializa Barômetro BMP280 (0x76)
  initBarometerBMP280();

  // 4. Calibração de Offset do Giroscópio e Referência do Barômetro (2000 amostras)
  Serial.println("Calibrando Giroscopio e Barometro... Mantenha o drone imovel.");
  for (int i = 0; i < 2000; i++) {
    readIMUSignals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;

    if (barometerAvailable) {
      readBarometerSignals();
      AltitudeBarometerStartUp += AltitudeBarometer;
    }
    delay(1);
  }
  RateCalibrationRoll  /= 2000.0f;
  RateCalibrationPitch /= 2000.0f;
  RateCalibrationYaw   /= 2000.0f;
  if (barometerAvailable) {
    AltitudeBarometerStartUp /= 2000.0f;
  }

  // 5. Inicializa Receptor PPM
  ReceiverInput.begin(PIN_PPM_RX);

  Serial.println("Setup concluido! Envie 'ARM' via Serial (com T < 1050) ou ative CH5 no Radio.");
  loopTimer = micros();
}

// ============================================================================
// 11. LOOP PRINCIPAL DE CONTROLE EM TEMPO REAL (250 Hz / 4000 us)
// ============================================================================
void loop() {
  // --- ETAPA 1: Aquisição e Calibração da IMU ---
  readIMUSignals();
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // --- ETAPA 2: Filtro de Kalman 1D para Ângulos de Atitude (Roll e Pitch) ---
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // --- ETAPA 3: Estimação de Altitude e Velocidade Vertical (Kalman 2D) ---
  float rollRad  = KalmanAngleRoll  * (PI / 180.0f);
  float pitchRad = KalmanAnglePitch * (PI / 180.0f);
  
  // Projeção da aceleração no eixo Z inercial (perpendicular ao solo)
  AccZInertial = -sin(pitchRad) * AccX 
                 + cos(pitchRad) * sin(rollRad) * AccY 
                 + cos(pitchRad) * cos(rollRad) * AccZ;
  // Subtrai 1g da gravidade e converte para cm/s^2
  AccZInertial = (AccZInertial - 1.0f) * 9.81f * 100.0f;

  if (barometerAvailable) {
    readBarometerSignals();
    AltitudeBarometer -= AltitudeBarometerStartUp;
    kalman_2d(AccZInertial, AltitudeBarometer);
  }

  // --- ETAPA 4: Leitura dos Comandos (Rádio PPM e Serial/Wi-Fi) ---
  readReceiverPPM();
  readSerialNonBlocking();

  // Conversão do comando de Guinada (sempre em taxa angular [deg/s], max +/- 75 deg/s)
  DesiredRateYaw = 0.15f * (ReceiverValue[3] - 1500.0f);

  // --- ETAPA 5: Malhas Externas de Controle (Conforme Modo Selecionado) ---
  if (currentMode == MODE_RATE) {
    // Modo Taxa Angular: sticks comandam diretamente deg/s (+/- 75 deg/s)
    DesiredRateRoll  = 0.15f * (ReceiverValue[0] - 1500.0f);
    DesiredRatePitch = 0.15f * (ReceiverValue[1] - 1500.0f);
    InputThrottle    = ReceiverValue[2];
  } 
  else {
    // Modos STABILIZE e ALT_HOLD: sticks comandam ângulos (+/- 50 graus)
    DesiredAngleRoll  = 0.10f * (ReceiverValue[0] - 1500.0f);
    DesiredAnglePitch = 0.10f * (ReceiverValue[1] - 1500.0f);

    // Malha Externa de Atitude (Ângulo -> Taxa Desejada)
    ErrorAngleRoll  = DesiredAngleRoll  - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll    = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch    = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    // Definição do Throttle (Manual no STABILIZE vs. Automático no ALT_HOLD)
    if (currentMode == MODE_ALT_HOLD && barometerAvailable) {
      // Se o stick de Throttle estiver na zona morta central (1450 a 1550 us),
      // mantém a altitude alvo (Hover Position Hold); fora dela, comanda velocidade vertical.
      if (ReceiverValue[2] > 1450.0f && ReceiverValue[2] < 1550.0f) {
        if (!holdAltitudeTargetLocked) {
          DesiredAltitude = AltitudeKalman;
          holdAltitudeTargetLocked = true;
        }
        // Malha Externa de Altitude (Erro de Altura [cm] -> Velocidade Vertical Desejada [cm/s])
        DesiredVelocityVertical = constrain(PPosZ * (DesiredAltitude - AltitudeKalman), -100.0f, 100.0f);
      } else {
        holdAltitudeTargetLocked = false;
        DesiredVelocityVertical = 0.3f * (ReceiverValue[2] - 1500.0f); // +/- 150 cm/s
      }

      // Malha Interna Vertical (Velocidade Vertical -> Comando de Throttle)
      ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
      pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical,
                   PrevErrorVelocityVertical, PrevItermVelocityVertical);
      InputThrottle             = THROTTLE_HOVER + PIDReturn[0];
      PrevErrorVelocityVertical = PIDReturn[1];
      PrevItermVelocityVertical = PIDReturn[2];
    } else {
      InputThrottle = ReceiverValue[2];
    }
  }

  // --- ETAPA 6: Malha Interna de Taxas Angulares (Rate PID Loop) ---
  ErrorRateRoll  = DesiredRateRoll  - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw   - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll         = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch         = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw         = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // Limita o Throttle base a 1800 us para garantir margem de manobra aos controles de atitude
  if (InputThrottle > THROTTLE_MAX) InputThrottle = THROTTLE_MAX;

  // --- ETAPA 7: Matriz de Mixagem dos Motores (Quadrimotor em X) ---
  // MDE (Pino 5 - Dianteiro Esquerdo) | MDD (Pino 6 - Dianteiro Direito)
  // MTE (Pino 4 - Traseiro Esquerdo)  | MTD (Pino 3 - Traseiro Direito)
  MotorInputMDD = InputThrottle - InputRoll - InputPitch - InputYaw; // Motor 1 (Front-Right)
  MotorInputMTD = InputThrottle - InputRoll + InputPitch + InputYaw; // Motor 2 (Back-Right)
  MotorInputMTE = InputThrottle + InputRoll + InputPitch - InputYaw; // Motor 3 (Back-Left)
  MotorInputMDE = InputThrottle + InputRoll - InputPitch + InputYaw; // Motor 4 (Front-Left)

  // Saturação superior (1999 us) e inferior em voo (ThrottleIdle = 1180 us)
  MotorInputMDD = constrain(MotorInputMDD, (float)THROTTLE_IDLE, (float)MOTOR_SAT_MAX);
  MotorInputMTD = constrain(MotorInputMTD, (float)THROTTLE_IDLE, (float)MOTOR_SAT_MAX);
  MotorInputMTE = constrain(MotorInputMTE, (float)THROTTLE_IDLE, (float)MOTOR_SAT_MAX);
  MotorInputMDE = constrain(MotorInputMDE, (float)THROTTLE_IDLE, (float)MOTOR_SAT_MAX);

  // --- ETAPA 8: Travas de Segurança (Throttle Cut & Anti-Windup no Solo) ---
  if (!motorsArmed || ReceiverValue[2] < 1050.0f) {
    MotorInputMDD = THROTTLE_CUTOFF;
    MotorInputMTD = THROTTLE_CUTOFF;
    MotorInputMTE = THROTTLE_CUTOFF;
    MotorInputMDE = THROTTLE_CUTOFF;
    reset_pid(); // Evita acúmulo de erro integral antes da decolagem
  }

  // --- ETAPA 9: Escrita nos Pinos PWM da PCB ---
  writeEscMicros(PIN_MDD, MotorInputMDD);
  writeEscMicros(PIN_MTD, MotorInputMTD);
  writeEscMicros(PIN_MTE, MotorInputMTE);
  writeEscMicros(PIN_MDE, MotorInputMDE);

  // --- ETAPA 10: Sincronismo Determinístico da Malha (250 Hz / 4000 us) ---
  while (micros() - loopTimer < LOOP_PERIOD_US);
  loopTimer = micros();
}