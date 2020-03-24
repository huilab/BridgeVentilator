//**** ANALOG INPUTS****
#define DELAY_AI 0
#define FREQUENCY_AI 1
#define VOLUME_AI 2

//**** DIGITAL INPUTS
#define LIMIT_ERROR  2  //Needed for interrupt
#define LIMIT_HOME 3  //Needed for interrupt

//**** DIGITAL OUTPUTS
#define DIRECTION 4
#define PWM_OUTPUT 5

bool ERROR_FLAG = false;

byte current_direction = 0;

unsigned short ai_low = 32; //10-bit input, leave some slop on the low end
unsigned short ai_high = 992; //10-bit input, leave some slop on the high end

//*** Analog Thresholds Speed ***
float speed_low = 33;  //Units in ms, Corresponds to 30Hz delay
float speed_high = 500; //Units in ms. Corresponds to 2Hz delay
float current_speed = 500;
float speed_slope = .4864; // In units of ms / bit
float speed_offset = 17.435; // Units of ms

//*** Analog Thresholds Volume ***
float volume_low = .2;  //Units in percent of speed duty cycle
float volume_high = 1.0; //Units in percent of speed duty cycle
float current_volume = 1.0;
float volume_slope =  8.33e-4; // In units of % duty cycle
float volume_offset = -31.999; // Units of ms


//*** Analog Thresholds Delay
unsigned short delay_low = 100; //100 ms delay after the interupt to reverse
unsigned short delay_high = 200; //200ms second delay
unsigned short current_delay = 10;



void limit_switch_home(){
  digitalWrite(PWM_OUTPUT, LOW);
}

void limit_switch_error(){
  digitalWrite(PWM_OUTPUT, LOW);
  ERROR_FLAG = true;
  //SOUND ALARM?
}

float map_analog_value(float input, float slope, float offset){
  return input*slope + offset;
}

void setup() {
  //Configure output pins
  pinMode(DIRECTION, OUTPUT);
  pinMode(PWM_OUTPUT, OUTPUT);

  //Configure Inputs
  pinMode(LIMIT_ERROR, INPUT);
  pinMode(LIMIT_HOME, INPUT);

  digitalWrite(DIRECTION, LOW);
  digitalWrite(PWM_OUTPUT, LOW);

  attachInterrupt(digitalPinToInterrupt(LIMIT_ERROR), limit_switch_error, RISING); //Limit switch forward hit, reverse direction
  attachInterrupt(digitalPinToInterrupt(LIMIT_HOME), limit_switch_home, RISING); //Limit switch back hit, reverse direction
}

void loop() {
  //First, read the analog inputs

  if(!ERROR_FLAG){
    unsigned short speed_val = analogRead(FREQUENCY_AI);
    speed_val = constrain(speed_val, ai_low, ai_high); //Constrain with the analog input bands
    current_speed = map_analog_value(speed_val, speed_slope, speed_offset);
    current_speed = constrain(current_speed, speed_low, speed_high);
  }

  if(!ERROR_FLAG){
    unsigned short volume_val = analogRead(VOLUME_AI);
    volume_val = constrain(volume_val, ai_low, ai_high); //Constrain with the analog input bands
    current_volume = map_analog_value(volume_val, volume_slope, volume_offset);
    current_volume = constrain(current_volume, volume_low, volume_high);
  }

  unsigned short duty_cycle = (unsigned short)current_speed*current_volume*1000;

  if(!ERROR_FLAG){
    digitalWrite(DIRECTION, HIGH);
    digitalWrite(PWM_OUTPUT, HIGH);
    delay(duty_cycle);
    digitalWrite(DIRECTION, LOW);
    digitalWrite(PWM_OUTPUT, HIGH);
  }

  delay(current_delay);
  ERROR_FLAG = false;
}
