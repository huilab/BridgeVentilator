/*!
 * Bridge Ventilator Software Demonstration
 * 
 * Hardware:
 * ESP32
 * SSD1306 128x64px OLED
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

template <typename T>
class SelectableScreen : public InputScreen<T> {
  bool m_selected = false;
  public:
    SelectableScreen(MenuScreen* parent, UiContext* context, const char* title, 
    T min_value = 0, T max_value = 100, T value_step = 1, char* suffix = nullptr              )
      : InputScreen<T>(parent, context, title, min_value, max_value, value_step, suffix) {
    }
    void setSelected(const bool is_selected) { 
      m_selected = is_selected;
      MenuScreen::setDirty(true);
      }
    void draw(UiContext* context) {
      if(m_selected) {
        context->display->drawFrame(0,0,128,64);
      }
      InputScreen<T>::draw(context);
    }
};

#define N_DEVICES 5
DeviceManager<N_DEVICES> device_manager;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C disp1(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
U8G2_SSD1306_128X64_NONAME_F_SW_I2C disp2(U8G2_R0, /* clock=*/ 23, /* data=*/ 19, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

UiContext context1(&disp1, 4);
UiContext context2(&disp2, 4);
Ui ui1(device_manager.registerDevice(), &context1);
Ui ui2(device_manager.registerDevice(), &context2);

LT_Encoder encoder(device_manager.registerDevice(), 33, 32, true);
LT_DebouncedButton button(device_manager.registerDevice(), 27, true);

MainMenu<7> screen_main1(NULL, &context1, "Menu");
MainMenu<7> screen_main2(NULL, &context2, "Menu");
SelectableScreen<float> screen_rate(&screen_main1, &context1, "Breath Rate", 2.0, 40.0, 1.0, " [1/min]");
SelectableScreen<float> screen_volume(&screen_main2, &context2, "Tidal Volume", 200.0, 800.0, 5.0, " [cc]");

SelectableScreen<float>* selected_screen = &screen_rate;

void IRAM_ATTR onEncoderInterrupt() {
    encoder.handleInterrupt();
}

void IRAM_ATTR onBtnInterrupt() {
    button.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  encoder.setValueChangedCallback(onEncoderValueChanged);
  encoder.setDebounceInterval(200000); // 100 ms
  button.setButtonReleasedCallback(onButtonReleased);
  button.setDebounceInterval(250000); // 100 ms

  ui1.setCurrentScreen(&screen_rate);
  ui2.setCurrentScreen(&screen_volume);
  
  device_manager.attachDevice(&encoder);
  device_manager.attachDevice(&button);
  device_manager.attachDevice(&ui1);
  device_manager.attachDevice(&ui2);

  attachInterrupt(digitalPinToInterrupt(32), onEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(33), onEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(27), onBtnInterrupt, CHANGE);

  screen_rate.setSelected(true);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  device_manager.update();
}

void onButtonReleased() {
  static bool is1 = true;
  
  is1=!is1;
  selected_screen = is1? &screen_rate : &screen_volume;
  screen_rate.setSelected(is1);
  screen_volume.setSelected(!is1);
  
  Serial.print("ctx");
  Serial.println(is1);
}

void onEncoderValueChanged() {
  static int8_t last_value = 0;
  int8_t value = encoder.position();
  int8_t delta = value - last_value;
  selected_screen->adjust(delta);
  /*
  if (delta > 0) {
    selected_screen->increment(delta);
    Serial.println("up");
  }
  else {
    selected_screen->decrement(delta);
    Serial.println("dn");
  }*/
  last_value = value;
}
