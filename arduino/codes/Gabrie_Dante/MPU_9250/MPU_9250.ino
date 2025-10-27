#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

#define PI  3.1415926535897932384626433
#define d2r PI/180//////////////////////////////////////////
#define r2d 180/PI/////////////////////////////////////////
//A4 - SDA
//A5 - SCL
//GND - ADO
//3.3V - VCC


float phi_filt = 0.0;
float theta_filt = 0.0;
float psi_filt = 0.0;

float phi_gyro = 0.0;
float theta_gyro = 0.0;
float psi_gyro = 0.0;

float phi_acc = 0.0;
float theta_acc = 0.0;

float phi_acc_filt = 0.0;
float theta_acc_filt = 0.0;

float alpha = 0.9;
float beta = 0.3;
float gamma = 0.9;

unsigned long lastTime = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();  // Accelerometer
  mySensor.beginGyro();  // Gyro
  mySensor.beginMag();  // Magnetometer

  if (!mySensor.accelUpdate() && !mySensor.gyroUpdate() && !mySensor.magUpdate()) {
    Serial.println("Sensor pronto!");
  } else {
    Serial.println("Erro ao inicializar o sensor");
    while (1);
  }
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();
  
  // Serial.print("Accel: ");
  // Serial.print(mySensor.accelX());
  // Serial.print(", ");
  // Serial.print(mySensor.accelY());
  // Serial.print(" , ");
  // Serial.println(mySensor.accelZ());

  // Serial.print("Gyro: ");
  // Serial.print(mySensor.gyroX());
  // Serial.print(", ");
  // Serial.print(mySensor.gyroY());
  // Serial.print(" , ");
  // Serial.println(mySensor.gyroZ());

  // Serial.print("Magnet: ");
  // Serial.print(mySensor.magX());
  // Serial.print(", ");
  // Serial.print(mySensor.magY());
  // Serial.print(" , ");
  // Serial.println(mySensor.magZ());

  

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime)/ 1000.0;  // em segundos
  lastTime = currentTime;

  phi_acc = atan2((mySensor.accelY()*9.95), (-mySensor.accelZ()*9.1))*r2d;
  theta_acc = atan2( -(mySensor.accelX()*9.8), sqrt(pow((mySensor.accelY()*9.95), 2) + pow((mySensor.accelZ()*9.1), 2)))*r2d;

  phi_acc_filt = beta*phi_acc_filt + (1-beta)*phi_acc;
  theta_acc_filt = beta*theta_acc_filt + (1-beta)*theta_acc;

  // Integração das velocidades angulares
  phi_gyro += (mySensor.gyroX()+                 0.585)*dt;
  theta_gyro += (mySensor.gyroY()+               0.527)*dt;
  psi_gyro += (mySensor.gyroZ()+                 0.89)*dt;


   //Filtro Complementar
  phi_filt = alpha*phi_gyro + (1-alpha)*phi_acc_filt;
  theta_filt = alpha*theta_gyro + (1-alpha)*theta_acc_filt;

  // Exibe os resultados
  Serial.print("Angulo ACC Phi com filtro: ");
  Serial.println(phi_acc_filt);
  Serial.print("Angulo ACC Theta com filtro: ");
  Serial.println(theta_acc_filt);
  
  Serial.print("Gyro x: ");
  Serial.println(mySensor.gyroX());
  Serial.print("Gyro y: ");
  Serial.println(mySensor.gyroY());

  Serial.print("Angulo theta gyro: ");
  Serial.println(theta_gyro);
  Serial.print("Angulo phi gyro: ");
  Serial.println(phi_gyro);

  Serial.print("Aceleracao X: ");
  Serial.println(mySensor.accelX()*9.71287129);
  Serial.print("Aceleracao Y: ");
  Serial.println(mySensor.accelY()*9.71287129);
  Serial.print("Aceleracao Z: ");
  Serial.println(mySensor.accelZ()*9); 


  // Mostra os dados do magnetômetro
  Serial.print("Mag X: ");
  Serial.print(mySensor.magX()+6.5);
  Serial.print(" | Y: ");
  Serial.print(mySensor.magY()-84.5);
  Serial.print(" | Z: ");
  Serial.println(mySensor.magZ()+20);
  Serial.println(" M: ");
  Serial.println(sqrt((mySensor.magX()+6.5)*(mySensor.magX()+6.5)+(mySensor.magY()-84.5)*(mySensor.magY()-84.5)+(mySensor.magZ()+20)*(mySensor.magZ()+20)));

  // Cálculo do azimute (ângulo em relação ao norte magnético)
  float heading = atan2((mySensor.magY()-84.5), (mySensor.magX()+6.5)) *r2d;
  //if (heading < 0) heading += 360;

  Serial.print("Azimute (graus): ");
  Serial.println(heading);
  Serial.println("--------------------------");

  delay(500);
}
