void setup() {
   
  pinMode(4, INPUT);
  pinMode(8, OUTPUT);
  blc_resetIRQ();
  delay(1000);
  blc_init();
}
 
void loop() {
  run_motor();
  //delay(50);
}
 
void start()
{
  Serial.begin(115200,SERIAL_8N1);
}
 
void pause()
{
  Serial.end();
  pinMode(1, INPUT);
}

void blc_init()
{
    // Initialisation multicast
    start();
    blc_reset();
    pause();
    delay(100);
      
    start();
    blc_getVersion();
    pause();
    delay(100);
      
    start();
    blc_run_motor();
    pause();
    delay(100);
    
    // Initialisation moteur
    start();
    Serial.write(2);
    pause();
    delay(100);
      
    start();
    Serial.write(0x40);
    pause();
    delay(100);
    
    delay(1000);
    
    blc_startled();
    
    delay(1000);
    //run_motor();
    //blc_stopled();
    
    //start();
    //blc_reset();
    //pause();
    //delay(100);
}
 
void blc_run_motor()
{
  Serial.write(0xA1);
}
 
void blc_reset()
{
  Serial.write(0xE0);
}
 
void blc_getVersion()
{
  Serial.write(0x91);
}
 
void blc_startled()
{
  start();
  Serial.write(0x60);
  pause();
  delay(100);
 
  start();
  Serial.write(0x1e);
  pause();
  delay(100);
     
}
 
void blc_stopled()
{
  start();
  Serial.write(0x60);
  pause();
  delay(100);
 
  start();
  Serial.write(0x00);
  pause();
  delay(100);
     
}
 
void blc_multicast()
{
  for (int i = 1; i <= 6; i++) {
  start();
  Serial.write(0xa0);
  pause();
  delay(100);
  }
}
 
void blc_resetIRQ()
{
     if (digitalRead(4) == HIGH) {
      pinMode(8, OUTPUT);
      digitalWrite(8, LOW);
      delay(500);
      pinMode(8, INPUT);
   }
}
 
void run_motor()
{
    start();
    Serial.write(47);
    pause();
    delay(5);
    
    start();
    Serial.write(247);
    pause();
    delay(5);
    
    start();
    Serial.write(251);
    pause();
    delay(5);
    
      start();
    Serial.write(253);
    pause();
    delay(5);
    
    start();
    Serial.write(254);
    pause();
    delay(5);
}