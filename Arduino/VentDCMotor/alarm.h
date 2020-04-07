#ifndef ALARM_H
#define ALARM_H

#include <Servo.h>
/*!
 * warning: the tone() function interferes with PWM on pins 3 and 11
*/ 
class BuzzerAlarm {
public:
  BuzzerAlarm(const uint8_t pin, const int freq = 1175, const int t_on = 1000, const int t_off = 500) : 
  M_PIN(pin), m_frequency(freq), m_t_on(t_on), m_t_off(t_off) {}

  void setup() {
    pinMode(M_PIN, OUTPUT);
  }

  void update(uint32_t now) {
    if(m_is_active) {
      // alarm is on
      if(m_is_noisy) {
        // buzzer making noise
        if( (now - m_last_update) >= m_t_on) {
          // time to stop noise
          noTone(M_PIN);
          m_is_noisy = false;
          m_last_update = now;
        }
        else {
          // keep making noise
        }
      }
      else {
        // busser is not making noise
        if( (now - m_last_update) >= m_t_off) {
          // time to start noise
          tone(M_PIN, m_frequency);
          m_is_noisy = true;
          m_last_update = now;
        }
        else {
          // stay silent for now
        }
      }
    }
    else {
      // alarm is not active
      if(m_is_noisy) {
        // buzzer making noise, stop noise
          noTone(M_PIN);
          m_is_noisy = false;
      }
      else {
        // no changes
      }
    }
  }

  // set the note to make noise at
  void setFrequency(int frequency) {
    m_frequency = frequency;
  }

  // turn the alarm on or off
  void setActive(bool is_active) {
    m_is_active = is_active;
  };

private:
  const int M_PIN; //< the pin the buzzer is attached to
  uint32_t m_last_update = 0; //< the time in ms of last change
  uint32_t m_t_on; //< time in ms to make noise
  uint32_t m_t_off; //< time in ms to shut up
  int m_frequency; //< the frequency of the tone to generate
  bool m_is_active = false; //< flag to track if alarm is sounding
  bool m_is_noisy = false; //< flag to track if currently making noise
};

#endif // ALARM_H
