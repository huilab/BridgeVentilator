/*
  VentDCMotor
  Driver for Bridge Ventilator automated bag valve mask emergency ventilator

  Hardware:
  Arduino UNO or MEGA
  Pin A0: Potentiometer, breath rate
  Pin A1: Potentiometer, arm speed control
  Pin A2: Potentiometer, volume control
  Pin 2: Limit switch, error
  Pin 3: Limit switch, home
  Pin 5, 4: Cytron motor driver
  Pin 9: Alarm buzzer

  Dependencies:
  https://github.com/CytronTechnologies/CytronMotorDriver

  License:

  Revision History:
  2020 03 27
  Added license
  Changed motor driver PWM pin from 3 to 5, moved error interrupt back to pin 3 for compatibility with UNO
  Changed variable macros to global constants

  2020 03 26
  Changed motor driver to Cytron
*/
#define DEBUG

// INPUTS
const int PIN_BREATH_RATE = A0;
const int PIN_ARM_SPEED = A1;
const int PIN_VOLUME = A2;
const int PIN_PRESSURE_SENSE = A3;
const int PIN_MAX_PRESSURE = A4;
const int PIN_LIMIT_ERROR  = 2;  // ISR 0
const int PIN_LIMIT_HOME = 3;  // ISR 1

// OUTPUTS
const int PIN_MOT_PWM = 5;
const int PIN_MOT_DIR = 4;
const int PIN_BUZZER = 9;

#include "vent_dc.h"
VentMotor motor(PWM_DIR, PIN_MOT_PWM, PIN_MOT_DIR);

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

// ISR functions
// the home limit switch has been pressed
// stop the motor
void onLimitSwitchHomePressed() {
  int pin_state = digitalRead(PIN_LIMIT_HOME);
  // the home switch is pulled to ground when pressed
  pin_state == LOW ? motor.setAtHome(true) : motor.setAtHome(false);
}

// Limit switch forward hit, stop the motor
void onLimitSwitchErrorPressed() {
  int pin_state = digitalRead(PIN_LIMIT_ERROR);
  // the limit switch is pulled to ground when pressed
  pin_state == LOW ? motor.setAtLimit(true) : motor.setAtLimit(false);
  error_code = LIMIT_ERROR;
  alarm.setActive(true);
}

template <typename T>
T mapFancy(T x, T in_min, T in_max, T out_min, T out_max) {
  if (in_max == in_min) {
    return x;
  }
  else {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}

void setup() {
#ifdef DEBUG
  // open serial port for debugging
  Serial.begin(9600);
#endif

  // Configure Inputs
  pinMode(PIN_LIMIT_ERROR, INPUT_PULLUP);
  pinMode(PIN_LIMIT_HOME, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_ERROR), onLimitSwitchErrorPressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_HOME), onLimitSwitchHomePressed, CHANGE);

  // move the motor home with 2 second timeout
  motor.goToHome(2000);

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
  const float V_SUPPLY = 5.0;
  const float P_MAX = -1.0;
  const float P_MIN = 1.0;
  const float P_RANGE = 1.0;

  // read current pressure value
  float counts = analogRead(PIN_PRESSURE_SENSE);
  float voltage = counts / 1024.0;
  float p = (voltage - 0.1 * V_SUPPLY * (P_MAX - P_MIN)) / (0.8 * V_SUPPLY) + P_MIN;
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
    motor.goToHome(2000);
    motor.setRunning(false);
#ifdef DEBUG
    Serial.print("High pressure alarm. Pressure: ");
    Serial.println(p);
#endif
  }
}

// read the analog input on pin, map and constrain it to the range, and store the result in value
void readPotentiometer(float &value, const int pin, const float range_low, const float range_high) {
  int counts = analogRead(pin);
  constrain(counts, ANALOG_MIN, ANALOG_MAX);
  value = mapFancy(static_cast<float>(counts), static_cast<float>(ANALOG_MIN), static_cast<float>(ANALOG_MAX), range_low, range_high);
#ifdef DEBUG
  Serial.print("Analog pin: ");
  Serial.print(pin);
  Serial.print("Counts: ");
  Serial.print(counts);
  Serial.print(" mapped to value: ");
  Serial.println(value);
#endif
}
