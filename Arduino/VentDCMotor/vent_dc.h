#ifndef VENT_DC_H
#define VENT_DC_H

#include "CytronMotorDriver.h"

/*!
*/
class VentMotor : public CytronMD {
  public:
    VentMotor(uint8_t mode, uint8_t pin1, uint8_t pin2) : CytronMD(mode, pin1, pin2) {}


    void update(const uint32_t &now) {
      if (m_running) {
        if (m_inhaling) {
          if ((now - m_last_update) >= m_t_inhale) {
            // inhale done, start exhale
            setSpeed(current_speed);
          }
        }
        else {
          if ((now - m_last_update) >= m_t_exhale) {
            // exhale done, start inhale
            setSpeed(-current_speed);
          }
        }
      }
      else {
        // do nothing if the motor has been stopped
      }
    }

    void setAtHome(const bool is_at_home) {
      m_at_home = is_at_home;
      if(m_at_home) {
        setSpeed(0);
      }
    }

    void setAtLimit(const bool is_at_limit) {
      m_at_limit = is_at_limit;
      if(m_at_limit) {
        setSpeed(0);
      }
    }

    // rotates motor ccw (negative speed) until limit switch is pressed or timeout
    // this function blocks until it is complete
    // returns true on success, false if timeout
    bool goToHome(const uint32_t timeout) {
      uint32_t start_time = millis();
      setSpeed(-current_speed);
      while ( (millis() - start_time) < timeout) {
        if (m_at_home) {
          return true;
        }
      }
      return false;
    }

    // rotates motor cw (positive speed) until limit switch is pressed or timeout
    // this function blocks until it is complete
    // returns true on success, false if timeout
    uint8_t goToLimit(const uint32_t timeout) {
      uint32_t now = millis();
      setSpeed(current_speed);
      while ( (millis() - now) < timeout) {
        if (m_at_home) {
          return true;
        }
      }
      return false;
    }
    
    void setRunning(const bool is_running) {
      m_running = is_running;
    }

  private:
    bool m_running = true;
    uint32_t m_last_update = 0; //< the time in ms of last change
    volatile bool m_at_home = false;
    volatile bool m_at_limit = false;
    bool m_inhaling = false;
    uint32_t m_t_inhale = 1000; // 1 s
    uint32_t m_t_exhale = 1000; // 1 s

    //*** Analog Thresholds Speed ***
    float arm_speed_low = 33;  //Units in ms
    float arm_speed_high = 500; //Units in ms
    float current_speed = 150;
    float arm_speed_slope = .2; // In units of ms / bit
    float arm_speed_offset = 35.0; // Units of ms

};

#endif // VENT_DC_H
