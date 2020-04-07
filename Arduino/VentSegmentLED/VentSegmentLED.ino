#include <LabThings.h>
#define N_DEVICES 5
DeviceManager<N_DEVICES> device_manager;

#include "TM1637.h"

// TM1736 pins
#define CLK 11
#define DIO 12

//TM1637Display display(CLK, DIO);
TM1637 display(CLK, DIO);

uint8_t modes[] { // 6 modes total
  0b01101101,    // (S)tart/stop
  0b01111111,    // (B)reath rate
  0b00111110,    // Breath (V)olume
  0b00110000,     // (I)nspiratory period
  0b01110110,    // e(X)piratory period
  0b01111001     // (E)rror
};

enum MODE {
  MODE_START, //0
  MODE_RATE, //1
  MODE_VOLUME, //2
  MODE_INSP, //3
  MODE_EXP, //4
  MODE_ERROR //5
};

int mode = 0;
bool is_editing = false;
bool is_running = false;

uint32_t t_last_blink;
bool is_blink;

float breath_rate = 40; //< Breaths per minute [units = 1/min (2 - 40)]
float breath_volume = 200; //< Breath volume [units = mL (200 - 800)}
float inhalation_period = 1.0; //< time to inhale [units = seconds]
float exhalation_period = 1.0; //< time to inhale [units = seconds]

float min_pressure = 20; //< max pressure [units = cm h2o]
float max_pressure = 40; //< max pressure [units = cm h2o]

struct VentParam {
  float value;
  float range_min;
  float range_max;
  float step_size;
  VentParam(float v, float r_min, float r_max, float s) : value(v), range_min(r_min), range_max(r_max), step_size(s) {}
};

VentParam params[] {
  VentParam(0, 0, 1, 1), // start/stop
  VentParam(40, 2, 40, 1),      // 0: rate
  VentParam(200, 200, 800, 10), // 1: volume
  VentParam(1.0, 0.5, 3.0, 0.5),// 2: i_p
  VentParam(1.0, 0.5, 3.0, 0.5),// 3: x_p
};

/*!
   Changes the variable stored at value by delta, 
   constrains to the range, and returns the updated value
*/
float updateParam(VentParam &p, const float delta)
{
  p.value += (p.step_size * delta);
  p.value = constrain(p.value, p.range_min, p.range_max);
  return p.value;
}


LT_Encoder encoder(device_manager.registerDevice(), 19, 20, true);
LT_DebouncedButton button(device_manager.registerDevice(), 21, true);

void setup() {
  Serial.begin(9600);

  // test display
  display.setBrightness(0x0f);
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  display.setSegments(data);
  delay(500);
  display.clear();
  updateDisplay();

  encoder.setValueChangedCallback(onEncoderValueChanged);
  encoder.setDebounceInterval(200000);
  button.setButtonReleasedCallback(onButtonReleased);

  device_manager.attachDevice(&encoder);
  device_manager.attachDevice(&button);

  attachInterrupt(digitalPinToInterrupt(19), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), onBtnChange, CHANGE);

}

void onEncoderChange() {
  encoder.handleInterrupt();
}

void onBtnChange() {
  button.handleInterrupt();
}

void loop() {
  device_manager.update();
  if (LT_current_time_us - t_last_blink > 400000) {
    t_last_blink = LT_current_time_us;
    is_blink = !is_blink;
    if (is_editing) {
      updateDisplay();
    }
  }
}

void onButtonReleased() {
  is_editing = !is_editing;
  Serial.println("btn");
  updateDisplay();
}

void onEncoderValueChanged() {
  static int last_value;
  int value = encoder.position();
  int delta = value - last_value;
  if (delta > 0) {
    encoderIncremented(delta);
  }
  else {
    encoderDecremented(delta);
  }
  last_value = value;
  updateDisplay();
}

void encoderIncremented(int delta)
{
  if (is_editing) {
    updateParam(params[mode], delta);
  }
  else {
    mode = mode + 1;
    if (mode > 4) {
      mode = 0;
    }
  }
}

void encoderDecremented(int delta)
{
  if (is_editing) {
    updateParam(params[mode], delta);
  }
  else {
    if (mode == 0) {
      mode = 4;
    }
    else {
      mode = mode - 1;
    }
  }
}

void updateDisplay()
{
  printDebug();

  //display.clear();
  if (is_blink && is_editing) {
    uint8_t data = 0b00000000;
    // blink the mode character
    display.setSegments(&data, 1, 0);
  }
  else {
    display.setSegments(&modes[mode], 1, 0);
  }
  // display current value
  float value = updateParam(params[mode], 0);
  display.showNumber(value, false, 3, 1);
}

void printDebug()
{
  float value = updateParam(params[mode], 0);
  Serial.print("mode:");
  Serial.print(mode);
  Serial.print("\t editing:");
  Serial.print(is_editing);
  Serial.print("\t value:");
  Serial.println(value);
}
