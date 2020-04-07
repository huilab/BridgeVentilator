//**** ANALOG INPUTS****
#define BREATH_RATE_AI 0
#define ARM_SPEED_AI 1
#define VOLUME_AI 2
#define POSITION_AI 3

//**** DIGITAL INPUTS
#define LIMIT_ERROR  2  //Needed for interrupt
#define LIMIT_HOME 18  //Needed for interrupt

//**** DIGITAL OUTPUTS
#define SSR1 4
#define SSR2 5
#define SSR3 6
#define SSR4 7

bool ERROR_FLAG = false;

byte current_direction = 0;

unsigned short ai_low = 32; //10-bit input, leave some slop on the low end
unsigned short ai_high = 992; //10-bit input, leave some slop on the high end

//*** Analog Thresholds Arm Speed ***
float arm_speed_low = 33;  //Units in ms
float arm_speed_high = 500; //Units in ms
float current_arm_speed = 500;
float arm_speed_slope = .4864; // In units of ms / bit
float arm_speed_offset = 17.435; // Units of ms

//*** Analog Thresholds Volume ***
float volume_low = .2;  //Units in percent of speed duty cycle
float volume_high = 1.0; //Units in percent of speed duty cycle
float current_volume = 1.0;
float volume_slope =  8.33e-4; // In units of % duty cycle
float volume_offset = -31.999; // Units of ms

//*** Analog Thresholds Breath Rate
float breath_rate_low = 30; //1 breath every 30 seconds
float breath_rate_high = 2; //1 breath every 2 seconds
float current_breath_rate = 10;
 

float position_home_voltage = 1.0;
float position_extended_voltage = 4.0;

bool CALIBRATION = false;
bool HAS_HOMED = false;
bool HAS_ERRORED = false;

void limit_switch_home(){
  brake();
  ERROR_FLAG = true;
  HAS_HOMED = true;
    //SOUND ALARM?
}

void limit_switch_error(){
  brake();
  ERROR_FLAG = true;
  HAS_ERRORED = true;
  //SOUND ALARM?
}

float map_analog_value(float input, float slope, float offset){
  return input*slope + offset;
}

float getPositionAsPercent(){
  float pos_volts = analogRead(POSITION_AI) * 5 / 1024;
  return abs(position_extended_voltage - position_home_voltage) / pos_volts;
}

void forward(byte velocity){
    analogWrite(SSR1, velocity); 
    analogWrite(SSR4, velocity);
    analogWrite(SSR2, 0); 
    analogWrite(SSR3, 0);
}

void reverse(byte velocity){
    analogWrite(SSR1, 0); 
    analogWrite(SSR4, 0);
    analogWrite(SSR2, velocity); 
    analogWrite(SSR3, velocity);
}

void brake(){
    analogWrite(SSR1, 0); 
    analogWrite(SSR2, 0);
    analogWrite(SSR3, 255); 
    analogWrite(SSR4, 255);
}

//Returns true if homed, or false if timeout
bool calibrate_home(){
    byte timeout = 0;
    while(!HAS_HOMED){
      if(timeout < 100){
        reverse(50);
        delay(50);
      }else{
        return false;
      }
      timeout += 1;
    }
    return true;
}

//Returns true if errored out, or false if timeout
bool calibrate_error(){
    byte timeout = 0;
    while(!HAS_ERRORED){
      if(timeout < 100){
        forward(50);
        delay(50);
      }else{
        return false;
      }
      timeout += 1;
    }
    return true;
}

void setup() {
  //Configure output pins
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);
  pinMode(SSR3, OUTPUT);
  pinMode(SSR4, OUTPUT);
  
  //Configure Inputs
  pinMode(LIMIT_ERROR, INPUT_PULLUP);
  pinMode(LIMIT_HOME, INPUT_PULLUP);

  brake();
    
  attachInterrupt(digitalPinToInterrupt(LIMIT_ERROR), limit_switch_error, FALLING); //Limit switch forward hit, reverse direction
  attachInterrupt(digitalPinToInterrupt(LIMIT_HOME), limit_switch_home, FALLING); //Limit switch back hit, reverse direction

  CALIBRATION = true;

  HAS_HOMED = false;
  HAS_ERRORED = false;

  while(CALIBRATION){
    //Called twice in case of a timeout (stuck at home or limit) in either case, need to cleanup later
    calibrate_home();  
    calibrate_error();
    calibrate_home();
    calibrate_error();    
  }
}

void loop() {
  //First, read the analog inputs

  float percent_pos = getPositionAsPercent();

  if(!ERROR_FLAG){
    unsigned short arm_speed_val = analogRead(ARM_SPEED_AI);
    arm_speed_val = constrain(arm_speed_val, ai_low, ai_high); //Constrain with the analog input bands
    current_arm_speed = map_analog_value(arm_speed_val, arm_speed_slope, arm_speed_offset);
    current_arm_speed = constrain(current_arm_speed, arm_speed_low, arm_speed_high);
  }

  if(!ERROR_FLAG){
    unsigned short volume_val = analogRead(VOLUME_AI);
    volume_val = constrain(volume_val, ai_low, ai_high); //Constrain with the analog input bands
    current_volume = map_analog_value(volume_val, volume_slope, volume_offset);
    current_volume = constrain(current_volume, volume_low, volume_high);
  }

  //Replace with encoder code unsigned short duty_cycle = (unsigned short)current_speed*current_volume*1000;

  if(!ERROR_FLAG){
    forward(255);
    delay(300);
    brake();
    delay(100);
    reverse(255);
    delay(285);  
    brake();
    delay(1000);
  }

  ERROR_FLAG = false;
}
