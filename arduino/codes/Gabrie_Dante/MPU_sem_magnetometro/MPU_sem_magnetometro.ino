#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;


#define PI  3.1415926535897932384626433
#define d2r PI/180//////////////////////////////////////////
#define r2d 180/PI/////////////////////////////////////////
//A4 - SDA
//A5 - SCL


float phi_filt = 0.0;
float theta_filt = 0.0;
float phi_gyro = 0.0;
float theta_gyro = 0.0;
float phi_acc = 0.0;
float theta_acc = 0.0;
float phi_acc_filt = 0.0;
float theta_acc_filt = 0.0;

float alpha = 0.9;
float beta = 0.3;


unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Inicializando MPU6050...");

  if (!mpu.begin(0x68)) {
    Serial.println("Falha ao encontrar MPU6050!");
    while (1);
  }

  Serial.println("MPU6050 pronto!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime)/ 1000.0;  // em segundos
  lastTime = currentTime;

  phi_acc = atan2(a.acceleration.y, a.acceleration.z );
  theta_acc = atan2( -a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)));
  


  phi_acc_filt = beta*phi_acc_filt + (1-beta)*phi_acc;
  theta_acc_filt = beta*theta_acc_filt + (1-beta)*theta_acc;

  // Integração das velocidades angulares
  phi_gyro += (g.gyro.x+(0.0112918027999*d2r))*dt;
  theta_gyro += (g.gyro.y+(1.966190063555*d2r))*dt;
  


  //Filtro Complementar

  phi_filt = alpha*phi_gyro + (1-alpha)*phi_acc_filt;
  theta_filt = alpha*theta_gyro + (1-alpha)*theta_acc_filt;


  // Exibe os resultados
  Serial.print("Angulo Phi com filtro: ");
  Serial.println(phi_filt*r2d);
  Serial.print("Angulo Theta com filtro: ");
  Serial.println(theta_filt*r2d);
  
  Serial.print("Gyro x: ");
  Serial.println(g.gyro.x*r2d);
  Serial.print("Gyro y: ");
  Serial.println(g.gyro.y*r2d);

  Serial.print("Angulo theta gyro: ");
  Serial.println(theta_gyro*r2d);
  Serial.print("Angulo phi gyro: ");
  Serial.println(phi_gyro*r2d);
 


  Serial.println("--------------------------");

  delay(50);
}
