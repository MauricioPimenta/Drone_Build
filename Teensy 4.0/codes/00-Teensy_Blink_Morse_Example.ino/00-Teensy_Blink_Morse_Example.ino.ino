/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  String word = "aabab";
}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {

  int i = 0;

  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(300);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(100);                  // wait for a second

  if (i >= 6)
    i = 0;
  
  
}

void charToMorseCode(char c, int* durations){

  switch(c){
    case 'a':
      durations[0] = 300;
      durations[1] = 1000;
      break;
    case 'b':
      durations[0] = 1000;
      durations[1] = 300;
      durations[2] = 300;
      durations[3] = 300;
      break;
    default:
      durations = 0;
  }

}

