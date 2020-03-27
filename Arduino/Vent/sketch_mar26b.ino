#include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.

//**** ANALOG INPUTS****
#define BREATH_RATE_AI 0
#define ARM_SPEED_AI 1
#define VOLUME_AI 2



//**** DIGITAL INPUTS
#define LIMIT_ERROR  2  //Needed for interrupt
#define LIMIT_HOME 18  //Needed for interrupt

//**** DIGITAL OUTPUTS


bool ERROR_FLAG = false;
bool break_flag = false;

bool CALIBRATION = false;
bool HAS_HOMED = false;
bool HAS_ERRORED = false;
bool reach_HOME=true;

byte current_direction = 0;

unsigned short ai_low = 32; //10-bit input, leave some slop on the low end
unsigned short ai_high = 992; //10-bit input, leave some slop on the high end

//*** Analog Thresholds Speed ***
float arm_speed_low = 33;  //Units in ms
float arm_speed_high = 500; //Units in ms
float current_speed = 150;
float arm_speed_slope = .2; // In units of ms / bit
float arm_speed_offset = 35.0; // Units of ms

//*** Analog Thresholds Volume ***
float volume_low = .2;  //Units in percent of speed duty cycle
float volume_high = 1.0; //Units in percent of speed duty cycle
float current_volume = 275;
float volume_slope =  1; // In units of % duty cycle
float volume_offset = 150; // Units of ms

//*** Analog Thresholds Delay
unsigned short delay_slope = 3.9; //100 ms delay after the interupt to reverse
unsigned short delay_intercept = 100; //200ms second delay
unsigned short delay_time = 1000;

void limit_switch_home() {

  motor.setSpeed(0); //brake

  ERROR_FLAG = true;
  HAS_HOMED = true;
    //SOUND ALARM?
}

void limit_switch_error() {
 
  motor.setSpeed(0); //brake

  ERROR_FLAG = true;
  HAS_ERRORED = true;
  //SOUND ALARM?
}

float map_analog_value(float input, float slope, float offset) {
  return input * slope + offset;
}

//Returns true if homed, or false if timeout

bool calibrate_home() {
  byte timeout = 0;
  while (!HAS_HOMED) {
    if (timeout < 100) {
      motor.setSpeed(-current_speed);
      delay(50);
    } else {
      return false;
    }
    timeout += 1;
  }
  return true;
}

//Returns true if errored out, or false if timeout
bool calibrate_error() {
  byte timeout = 0;
  while (!HAS_ERRORED) {
    if (timeout < 100) {
      motor.setSpeed(150);
      delay(50);
    } else {
      return false;
    }
    timeout += 1;
  }
  return true;
}


void setup() {

  //Configure Inputs
  //pinMode(LIMIT_ERROR, INPUT);
  pinMode(LIMIT_HOME, INPUT_PULLUP);

  Serial.begin(9600);
  //start program
  //attachInterrupt(digitalPinToInterrupt(LIMIT_ERROR), limit_switch_error, RISING); //Limit switch forward hit, reverse direction
  attachInterrupt(digitalPinToInterrupt(LIMIT_HOME), limit_switch_home, FALLING);

  motor.setSpeed(255); //FORWARD
  delay(1000);
  CALIBRATION = true;

  HAS_HOMED = false;
  HAS_ERRORED = false;

  while (CALIBRATION) {
    //Called twice in case of a timeout (stuck at home or limit) in either case, need to cleanup later
    CALIBRATION =!calibrate_home();
    // calibrate_home();
    // calibrate_error();
  }


}

void loop() {

  //First, read the analog inputs

    if (!ERROR_FLAG) {
      unsigned short volume_val = analogRead(VOLUME_AI);
      volume_val = constrain(volume_val, ai_low, ai_high); //Constrain with the analog input bands
      current_volume = volume_val * (volume_slope) + volume_offset;; //Constrain with the analog input bands
      //current_volume = map_analog_value(volume_val, volume_slope, volume_offset);
      current_volume = constrain(current_volume, volume_low, volume_high);
      //Serial.println("Analog value on the volume  is ");
      //Serial.println(current_volume);
      //Serial.println("\n ");
    }
  
  
    if (!ERROR_FLAG) {
      unsigned short speed_val = analogRead(BREATH_RATE_AI);
      speed_val = constrain(speed_val, ai_low, ai_high); //Constrain with the analog input bands
      delay_time = speed_val * (delay_slope) + delay_intercept;
      
      //Serial.println("Analog value on the rate in milliseconds is ");
      //Serial.println(delay_time);
      //Serial.println("\n ");
  
      speed_val = analogRead(ARM_SPEED_AI);
      speed_val = constrain(speed_val, ai_low, ai_high); //Constrain with the analog input bands
      current_speed = speed_val * (delay_slope) + delay_intercept;
    }
  
  
  
    if (!ERROR_FLAG) {
      motor.setSpeed(current_speed); //FORWARD
      delay(current_volume);
  
      motor.setSpeed(0); //STOP
      delay(100);
  
      reach_HOME=true;
      HAS_HOMED = false;
      while (reach_HOME) {
           reach_HOME =!calibrate_home();
      }
      motor.setSpeed(0); //STOP
      delay(delay_time);
    }
  
  
    //delay(current_delay);
    ERROR_FLAG = false;
}
