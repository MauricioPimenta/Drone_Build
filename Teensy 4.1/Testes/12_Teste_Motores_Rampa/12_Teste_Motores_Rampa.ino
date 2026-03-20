/*
  This code dont use the potentiometer anymore. It makes the throtle increase linearly and then
  decrease linearly so we can test all motor with different speeds.

  Teensy 4.1 throttle -> ESC (servo PWM) using BC548 inverter (open-collector)

  Wiring:
  - Pot:
      3.3V ---- pot end
      GND  ---- other pot end
      A0 (pin 14) ---- pot wiper (middle)
      (You added a capacitor from A0 to GND: good, keep it)
  - ESC signal via BC548 (inverted):
      Teensy escPin -> Rbase (1k..4.7k, e.g. 2.2k) -> BC548 Base
      BC548 Emitter -> GND
      BC548 Collector -> ESC signal wire
      Teensy GND <-> ESC GND (mandatory)

  Signal logic is inverted:
    Teensy LOW  => transistor OFF => ESC signal pulled HIGH (by ESC pull-up to 5V)
    Teensy HIGH => transistor ON  => ESC signal LOW
*/

#include <Arduino.h>

/* 
 * Motor Pins are now soldered in the PCB
 * Pin 3 = Motor traseiro direito
 * Pin 4 = Motor traseiro esquerdo
 * Pin 5 = Motor dianteiro esquerdo
 * Pin 6 = Motor dianteiro direito
 */
const int motor_traseiro_direito = 3;       // choose a digital pin for ESC control (any normal GPIO works)
const int motor_traseiro_esquerdo = 4;
const int motor_dianteiro_direito = 5;
const int motor_dianteiro_esquerdo = 6;

// const int LED_red = 0;
// const int LED_green = 1;
// const int LED_blue = 2;


constexpr uint32_t PERIOD_US = 20000;  // 50 Hz
constexpr int PULSE_MIN_US = 1000;     // typical ESC min
constexpr int PULSE_MAX_US = 2000;     // typical ESC max

// Helps arming: keep minimum throttle at boot
constexpr uint32_t ARM_TIME_MS = 3000;

// Filtering (simple IIR low-pass)
// Bigger SHIFT => more smoothing (slower response).
// SHIFT=4 => alpha = 1/16 (good start)
constexpr int FILTER_SHIFT = 4;

// Optional deadband near minimum to prevent tiny noise from spinning motor
constexpr int DEAD_BAND_US = 20;

// If you want to limit max throttle (safety), reduce this (e.g. 1700)
constexpr int SAFETY_MAX_US = PULSE_MAX_US;

// ----- Internal state -----
elapsedMillis tick;
int filteredPulseUs = PULSE_MIN_US;


/*
 * Inverted pulse generator: keeps ESC line HIGH for high_us, then LOW for rest of period.
 */
static inline void sendEscPulseInverted(int high_us) {
  high_us = constrain(high_us, PULSE_MIN_US, SAFETY_MAX_US);

  // ESC line HIGH:
  // (inverted) -> Teensy LOW turns transistor OFF -> ESC pull-up makes line HIGH
  digitalWriteFast(motor_dianteiro_direito, LOW);
  digitalWriteFast(motor_dianteiro_esquerdo, LOW);
  digitalWriteFast(motor_traseiro_direito, LOW);
  digitalWriteFast(motor_traseiro_esquerdo, LOW);
  delayMicroseconds(high_us);

  
  // ESC line LOW for the remainder:
  // Teensy HIGH turns transistor ON -> pulls line LOW
  digitalWriteFast(motor_dianteiro_direito, HIGH);
  digitalWriteFast(motor_dianteiro_esquerdo, HIGH);
  digitalWriteFast(motor_traseiro_direito, HIGH);
  digitalWriteFast(motor_traseiro_esquerdo, HIGH);
  delayMicroseconds((int)PERIOD_US - high_us);
}


/* ---- SETUP  ----*/
void setup() {
  // Teensy 4.1 ADC is 12-bit capable
  analogReadResolution(12);          // 0..4095
  analogReadAveraging(8);            // extra smoothing in hardware (optional, but helpful)

  pinMode(motor_traseiro_direito, OUTPUT);
  pinMode(motor_traseiro_esquerdo, OUTPUT);
  pinMode(motor_dianteiro_direito, OUTPUT);
  pinMode(motor_dianteiro_esquerdo, OUTPUT);

  // Idle state: keep ESC line HIGH (transistor OFF)
  digitalWriteFast(motor_dianteiro_direito, LOW);
  digitalWriteFast(motor_dianteiro_esquerdo, LOW);
  digitalWriteFast(motor_traseiro_direito, LOW);
  digitalWriteFast(motor_traseiro_esquerdo, LOW);

  // LED
  // pinMode(LED_red, OUTPUT);
  // pinMode(LED_green, OUTPUT);
  // pinMode(LED_blue, OUTPUT);

  // digitalWrite(LED_red, HIGH);

  // Let things power up
  delay(500);

  // Arm ESC at minimum throttle for a while
  uint32_t t0 = millis();
  while (millis() - t0 < ARM_TIME_MS) {
    sendEscPulseInverted(PULSE_MIN_US);
  }

  // Initialize filter at min
  filteredPulseUs = PULSE_MIN_US;
}

int raw = 0;
bool inc = true;


/* ---- LOOP ---- */
void loop() {
  
  // Check time passed to increment motor velocity
  if (tick > 500)
  {
    if (inc) // increasing speed
    {
      raw++;

      if (raw > 4095)
        inc = false;

    }
    else // decreasing speed
    {
      raw--;
      if (raw < 0)
        inc = true;
    }

    raw = constrain(raw, 0, 4095);
  }

  // 2) Map to pulse width
  int targetPulseUs = map(raw, 0, 4095, PULSE_MIN_US, PULSE_MAX_US);

  // 3) Deadband near min (helps prevent unintended spin from noise)
  if (targetPulseUs < (PULSE_MIN_US + DEAD_BAND_US)) {
    targetPulseUs = PULSE_MIN_US;
  }

  // 4) IIR filter: filtered += (target - filtered)/2^SHIFT
  filteredPulseUs += (targetPulseUs - filteredPulseUs) >> FILTER_SHIFT;

  // 5) Output one ESC frame (20ms total)
  sendEscPulseInverted(filteredPulseUs);

  // loop repeats at ~50Hz because sendEscPulseInverted already delays ~20ms
  
}