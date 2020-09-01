/*!
 * Bridge Ventilator Software Demonstration
 * 
 * Hardware:
 * Arduino Mega
 * KS0108 128x64px GLCD
 * Rotary Encoder with pushbutton
 * Piezo Buzzer
 * 1/4 Scale Hobby Servo
 * 
 * Version 1.0
 * by Erik Werner 2020 04 06
 * 
 * License: GPLv3
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <LabThings.h>
#include "vent_servo.h"

/// Define parameter limits
const float MIN_IP = 0.5;
const float MIN_XP = 0.5;
const float MIN_RR = 2;
const float MAX_RR = 40;
const float MIN_VOL = 200.0;
const float MAX_VOL = 800.0;


#include <HoneywellTruStabilitySPI.h>
/**************** Wrapper Class ****************/
/* Define a class that inherits from the sensor's library and from LT_Sensor 
 * Using LT_Sensor instead of LT_Device adds polling and data callback features 
*/
class LT_SPISensor : public TruStabilityPressureSensor, public LT_Sensor {

  public:
    // The constructor calls the sensor library constructor
    LT_SPISensor(const int id, uint8_t ss_pin, float p_min, float p_max)  :
    TruStabilityPressureSensor(ss_pin, p_min, p_max), LT_Sensor(id) {}

    // Subclasses of LT_Device must implement type()
    LT::DeviceType type() const { return (LT::DeviceType)(LT::UserType + 1); }

    // begin() implements library-specific initialization 
    void begin() {
      TruStabilityPressureSensor::begin();
    }

    // override readSensor() to let the Lab Things device manager poll the sensor
    uint8_t readSensor() {
      // TruStabilityPressureSensor::readSensor() returns 0 when new data is available
      // when readSensor() returns 0 to the base class, the new data callback is executed
      return TruStabilityPressureSensor::readSensor();
    }
    
};

#define SLAVE_SELECT_PIN SS
TruStabilityPressureSensor sensor( SLAVE_SELECT_PIN, -15.0, 15.0 );

// a device manager with support for up to 8 devices
DeviceManager<8> device_manager;

LT_Buzzer buzzer(device_manager.registerDevice(), 9);

// the mesenger handles sends ascii messages over the serial port
ASCIISerial messenger(Serial);

// the message handler allows callbacks to be attached to messages
MessageHandler handler;

// constructor for a 128x64px KS0108 GLCD display
U8G2_KS0108_128X64_F u8g2(U8G2_R2, 22, 23, 24, 25, 26, 27, 28, 29, /*en=*/ 34, /*dc=*/ 35, /*cs0=*/ 39, /*cs1=*/ 38, /*cs2=*/ U8X8_PIN_NONE, /* reset=*/  40); // Set R/W to low!

// create a context object from the display object
UiContext context(&u8g2);

// declare the user interface device and register with device manager
Ui ui(device_manager.registerDevice(), &context);

// a rotary encoder on pins 19 adn 20 with pullups enabled
LT_Encoder encoder(device_manager.registerDevice(), 19, 20, true);

// a pushbutton (part of the encoder) on pin 21 with pullup enabled
LT_DebouncedButton button(device_manager.registerDevice(), 21, true);

// declare all the ui elements
MainMenu<7> screen_main(NULL, &context, "Main Menu", 0x41);

MenuScreen screen_dashboard(&screen_main, &context, "Dashboard");
GraphItem<float, 32> graph(&screen_dashboard, "Pressure", "t(s)", "Press.(psi)", 64, 0, 64, 42);
NumberItem<float> dash_rate(&screen_dashboard, context.getFontSmall(), 34, 0, 16, 8, MIN_RR, MAX_RR);
NumberItem<float> dash_vol(&screen_dashboard, context.getFontSmall(), 34, 12, 16, 8, MIN_VOL, MAX_VOL);
NumberItem<float> dash_ip(&screen_dashboard, context.getFontSmall(),34, 24, 16, 8, MIN_IP, 5.0);
NumberItem<float> dash_xp(&screen_dashboard, context.getFontSmall(), 34, 36, 16, 8, MIN_XP, 5.0);
TextItem<16> text_rate(&screen_dashboard, context.getFontSmall(), "Rate");
TextItem<16> text_vol(&screen_dashboard, context.getFontSmall(), "Volume");
TextItem<16> text_ip(&screen_dashboard, context.getFontSmall(), "I.Time");
TextItem<16> text_xp(&screen_dashboard, context.getFontSmall(), "E.Time");
TextItem<16> text_status(&screen_dashboard, context.getFontSmall(), "System Stopped", AlignHCenter);
LineItem dash_line(&screen_dashboard, 0, 54, 128, 54);

InputScreen<int> screen_start(&screen_main, &context, "Start/Stop", 0, 1);
InputScreen<float> screen_rate(&screen_main, &context, "Breath Rate", MIN_RR, MAX_RR, 1.0, " [1/min]");
InputScreen<float> screen_volume(&screen_main, &context, "Tidal Volume", MIN_VOL, MAX_VOL, 10.0, " [cc]");
InputScreen<float> screen_ip(&screen_main, &context, "Insp. Time", MIN_IP, 5.0, 0.1, " [s]");
InputScreen<float> screen_xp(&screen_main, &context, "Exp. Time", MIN_XP, 5.0, 0.1, " [s]");
MainMenu<7> screen_setup(&screen_main, &context, "Setup");
InputScreen<int> screen_home(&screen_setup, &context, "Set Home", 1000, 2000, 50, " [us]");
InputScreen<int> screen_exhale(&screen_setup, &context, "Set Exhale Stop", 1000, 2000, 50, " [us]");
InputScreen<int> screen_inhale(&screen_setup, &context, "Set Inhale Stop", 1000, 2000, 50, " [us]");

VentServo servo(device_manager.registerDevice(), 5, A5);

LT_SPISensor pressure_sensor(device_manager.registerDevice(), SS, -15.0, 15.0);

// ISR
void onEncoderInterrupt() {
  encoder.handleInterrupt();
}

void onBtnInterrupt() {
  button.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  encoder.setValueChangedCallback(onEncoderValueChanged);
  encoder.setDebounceInterval(200000);
  button.setButtonPressedCallback(onButtonReleased);
  button.setDebounceInterval(250000);
  
  ui.setCurrentScreen(&screen_main);
  ui.setScreenSaverEnabled(false);

  screen_start.setValueChangedCallback(onMotorStartStop);
  screen_start.setValue(0);
  
  screen_volume.setValueChangedCallback(onVolumeChanged);
  screen_volume.setValue(600);
  screen_volume.setPrecision(1);

  screen_rate.setValueChangedCallback(onRateChanged);
  screen_rate.setValue(30);
  screen_rate.setPrecision(1);

  screen_ip.setValueChangedCallback(onInhalePeriodChanged);
  screen_ip.setValue(1.0);
  screen_ip.setPrecision(1);

  screen_xp.setValueChangedCallback(onExhalePeriodChanged);
  screen_xp.setValue(2.0);
  screen_xp.setPrecision(1);

  screen_home.setValueChangedCallback(onHomeChanged);
  screen_home.setScreenEnteredCallback(onHomeChanged);
  screen_home.setValue(2000, false);

  screen_exhale.setValueChangedCallback(onExhaleChanged);
  screen_exhale.setScreenEnteredCallback(onExhaleChanged);
  screen_exhale.setValue(1750, false);

  screen_inhale.setValueChangedCallback(onInhaleChanged);
  screen_inhale.setScreenEnteredCallback(onInhaleChanged);
  screen_inhale.setValue(1000, false);

  text_rate.setPos(0, 8);
  text_vol.setPos(0, 20);
  text_ip.setPos(0, 32);
  text_xp.setPos(0, 44);
  text_status.setPos(0, 62);
  screen_dashboard.addChild(&dash_rate);
  screen_dashboard.addChild(&dash_vol);
  screen_dashboard.addChild(&dash_ip);
  screen_dashboard.addChild(&dash_xp);
  screen_dashboard.addChild(&text_rate);
  screen_dashboard.addChild(&text_vol);
  screen_dashboard.addChild(&text_ip);
  screen_dashboard.addChild(&text_xp);
  screen_dashboard.addChild(&text_status);
  screen_dashboard.addChild(&graph);
  screen_dashboard.addChild(&dash_line);
  
  screen_main.addScreen(&screen_dashboard);
  screen_main.addScreen(&screen_start);
  screen_main.addScreen(&screen_rate);
  screen_main.addScreen(&screen_volume);
  screen_main.addScreen(&screen_ip);
  screen_main.addScreen(&screen_xp);
  screen_main.addScreen(&screen_setup);
  
  screen_setup.addScreen(&screen_home);
  screen_setup.addScreen(&screen_exhale);
  screen_setup.addScreen(&screen_inhale);

  device_manager.attachDevice(&buzzer);
  device_manager.attachDevice(&encoder);
  device_manager.attachDevice(&button);
  device_manager.attachDevice(&ui);
  device_manager.attachDevice(&servo);
  device_manager.attachDevice(&pressure_sensor);

  attachInterrupt(digitalPinToInterrupt(19), onEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), onEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), onBtnInterrupt, CHANGE);

  messenger.setMessageReceivedCallback(onMessageReceived);
  // setup message handler callbacks
  handler.attachFunction(1, onGoTo);

  graph.setGraphType(LT::LineGraph);
  
  pressure_sensor.setPolling(true);
  pressure_sensor.setPollingInterval(100000); // 10ms
  pressure_sensor.setNewDataCallback(onNewSensorData);

  // setup timer0 interrupt (gets called every 1 ms)
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  SPI.begin(); // start SPI communication
  sensor.begin(); // run sensor initialization

  Serial.println("Startup complete");
}

SIGNAL(TIMER0_COMPA_vect) 
{
  LT_current_time_us = micros();
  servo.lerpPw();
}

void loop() {
  messenger.update();
  device_manager.update();
}

// simulate sampling pressure data for graph
void onNewSensorData() {
  float p_cmh2o = pressure_sensor.pressure() * 70.307;
  float ts = pressure_sensor.lastSampleTime()/1000000.0;
  graph.addDataPoint(ts, p_cmh2o);
  Serial.println(p_cmh2o);
}

// send the message id to the handler
void onMessageReceived(int msg_id) {
  handler.handleMessage(msg_id);
  static bool is_active = false;
  is_active = ! is_active;
  Serial.print("buzzer?");
  Serial.println(is_active);
  buzzer.setActive(is_active);
}

void onGoTo() {
  int pw = messenger.getNextArgInt();
  servo.setPulseWidth(pw);
  Serial.print("Set pulse width to:");
  Serial.println(pw);
}

//*****Callbacks for parameter changes*****//
void onRateChanged() {
  float rate = screen_rate.value();
  float ip = screen_ip.value();
  float xp = screen_xp.value();
  if( (ip+xp) > 60.0/rate ) {
    // if rate is too fast, scale ip and xp to maintain i:e
    float old_total = ip + xp;
    float new_total = (ip + xp) - (60.0/rate);
    float dif = old_total - new_total;
    float ratio = dif / old_total;
    ip = ip*ratio;
    xp = xp*ratio;
    screen_ip.setValue(ip);
    //onInhalePeriodChanged();
    screen_xp.setValue(xp);
    //onExhalePeriodChanged();
  }
  servo.setBreathRate(rate);
  dash_rate.setValue(rate);
}

void onVolumeChanged() {
  float volume = screen_volume.value();
  servo.setVolume(volume);
  dash_vol.setValue(volume);
}

void onInhalePeriodChanged() {
  float rate = screen_rate.value();
  float ip = screen_ip.value();
  float xp = screen_xp.value();
  if( (ip+xp) > 60.0/rate ) {
    // limit ip so it doesn't interfere with rate
    ip = 60.0/rate - xp;
    screen_ip.setValue(ip, false);
  }
  dash_ip.setValue(ip);
  servo.setInspiratoryPeriod(ip);
}

void onExhalePeriodChanged() {
  float rate = screen_rate.value();
  float ip = screen_ip.value();
  float xp = screen_xp.value();
  if( (ip+xp) > 60.0/rate ) {
    // limit xp so it doesn't interfere with rate
    xp = 60.0/rate - ip;
    screen_xp.setValue(xp, false);
  }
  dash_xp.setValue(xp);
  servo.setExpiratoryPeriod(xp);
}

//*****Callbacks for position calibration*****//
void onHomeChanged() {
  screen_start.setValue(0);
  int value = screen_home.value();
  servo.calibrateHome(value);
}

void onExhaleChanged() {
  screen_start.setValue(0);
  int value = screen_exhale.value();
  servo.calibrateExhaleEnd(value);
}

void onInhaleChanged() {
  screen_start.setValue(0);
  int value = screen_inhale.value();
  servo.calibrateInhaleEnd(value);
}

void onMotorStartStop() {
  int state = screen_start.value();
  servo.setRunning(state);
}

void onButtonReleased() {
  ui.enter();
}

void onEncoderValueChanged() {
  static int8_t last_value;
  int8_t value = encoder.position();
  int8_t delta = value - last_value;
  ui.adjust(delta);
  //update menu
  /*if (value > last_value) {
    ui.increment();
  }
  else {
    ui.decrement();
  }*/
  last_value = value;
}

void onError() {
  Serial.println("Error");
}
