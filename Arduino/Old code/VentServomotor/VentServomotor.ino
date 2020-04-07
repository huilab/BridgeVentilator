/*
  Vent
  Driver for Bridge Ventilator automated bag valve mask emergency ventilator

  Hardware:
  Arduino UNO or MEGA
  Pin A0: Potentiometer, breath rate
  Pin A1: Potentiometer, arm speed control
  Pin A2: Potentiometer, volume control
  Pin A3: Servo feedback
  Pin 5: Servo PWM
  Pin 9: Alarm buzzer



  License:

  Revision History:
  2020 03 27
  First draft
*/

#define DEBUG

// INPUTS
const int PIN_BREATH_RATE = A0;
const int PIN_ARM_SPEED = A1;
const int PIN_VOLUME = A2;
const int PIN_PRESSURE_SENSE = A3;
const int PIN_MAX_PRESSURE = A4;

// OUTPUTS
const int PIN_MOT_PWM = 5;
const int PIN_MOT_FB = A5;
const int PIN_BUZZER = 9;

#include "vent_servo.h"
VentServo motor(PIN_MOT_PWM, PIN_MOT_FB);

#include "alarm.h"
BuzzerAlarm alarm(PIN_BUZZER);

// leave some slop around the top and bottom of the ADC (10-bit, 0-1023)
const int ANALOG_MIN = 32;
const int ANALOG_MAX = 992;

// define status codes
enum ERROR_CODE {
  NO_ERROR = 0,
  NO_HOME = 1,
  TIMEOUT_ERROR = 2,
  LIMIT_ERROR = 4,
  UNDEFINED_ERROR = 8
};

//**** global variables ****//

// startup without home position information
volatile ERROR_CODE error_code = NO_HOME;

float breath_rate = 40; //< Breaths per minute [units = 1/min (2 - 40)]
float breath_volume = 200; //< Breath volume [units = mL (200 - 800)}
float inhalation_period = 1.0; //< time to inhale [units = seconds]
float min_pressure = 20; //< max pressure [units = cm h2o]
float max_pressure = 40; //< max pressure [units = cm h2o]

//**************************//

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) {
    return x;
  }
  else {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}

void setup() {
  // open serial port for debugging
  Serial.begin(9600);
  alarm.setup();
  motor.setup();
}

void loop() {
  uint32_t now = millis();
  checkPressure();
  readPotentiometer(breath_rate, PIN_BREATH_RATE, 20.0, 40.0);
  readPotentiometer(inhalation_period, PIN_ARM_SPEED, 0.1, 3.0);
  readPotentiometer(breath_volume, PIN_VOLUME, 200.0, 800.0);
  readPotentiometer(max_pressure, PIN_MAX_PRESSURE, 0.0, 50.0);
  motor.update(now);
  alarm.update(now);
}

void checkPressure() {
  // read current pressure value
  float p;
  // sound alarm if underpressure
  if (p < min_pressure) {
    alarm.setActive(true);
#ifdef DEBUG
    Serial.print("Low pressure alarm. Pressure: ");
    Serial.println(p);
#endif
  }
  // sound alarm and stop motor if overpressure
  if (p > max_pressure) {
    alarm.setActive(true);
    motor.goHome();
#ifdef DEBUG
    Serial.print("High pressure alarm. Pressure: ");
    Serial.println(p);
#endif
  }
}

void readPotentiometer(float &value, const int pin, const float range_low, const float range_high) {
  int counts = analogRead(counts);
  constrain(counts, ANALOG_MIN, ANALOG_MAX);
  value = mapFloat(counts, ANALOG_MIN, ANALOG_MAX, range_low, range_high);
#ifdef DEBUG
  Serial.print("Analog pin: ");
  Serial.print(pin);
  Serial.print("Counts: ");
  Serial.print(counts);
  Serial.print(" mapped to value: ");
  Serial.println(value);
#endif
}
