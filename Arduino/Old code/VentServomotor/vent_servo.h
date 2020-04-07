#ifndef VENT_SERVO_H
#define VENT_SERVO_H

#include <Servo.h>

/*!
*/
class VentServo{
  public:
  VentServo(const uint8_t servo_pin, const uint8_t feedback_pin) 
  : M_SERVO_PIN(servo_pin), M_FEEDBACK_PIN(feedback_pin) {}

  void setup() {
    pinMode(M_SERVO_PIN, OUTPUT);
    m_servo.attach(M_SERVO_PIN, 1000, 2000);
    pinMode(M_FEEDBACK_PIN, INPUT);
  }

  void update(const uint32_t &now) {
    if(m_running) {
      
    }
    else {
      // do nothing if the motor has been stopped
    }
  }

  // instructs the servo to move to the angle
  // angle: the position in degrees
  void goToAngle(const int angle)
  {
    // this assumes the motor has a 180 range
    int pw = map(angle, 0, 180, 1000, 2000);
    setPulseWidth(pw);
  }

  
  void setPulseWidth(const int pw)
  {
    m_servo.writeMicroseconds(pw);
  }

  void goHome() {
    m_servo.writeMicroseconds(m_position_home);
  }

  void getSetPosition() {
    return m_servo.readMicroseconds();
  }
  
  void getFeedbackPosition() {
    return analogRead(M_FEEDBACK_PIN);
  }

  // the current position in counts is saved as the 
  void calibrateHome(const int value) {
    m_position_home = value;
    m_servo.writeMicroseconds(value);
    m_feedback_home = analogRead(M_FEEDBACK_PIN);
  }
  
  // saves the current position in counts as the servo exhale stop
  void calibrateExhaleEnd(const int value) {
    m_position_exhale_end = value;
    m_servo.writeMicroseconds(value);
    m_feedback_exhale_end = analogRead(M_FEEDBACK_PIN);
  }

  // saves the current position in counts as the servo inhale stop
  void calibrateInhaleEnd(const int value) {
    m_position_inhale_end = value;
    m_servo.writeMicroseconds(value);
    m_feedback_inhale_end = analogRead(M_FEEDBACK_PIN);
  }

  private:
    Servo m_servo;
    bool m_running = true;
    const int M_SERVO_PIN; //< the pin the servo is attached to
    const int M_FEEDBACK_PIN; //< the pin the servo is attached to
    int m_position_home = -1; //< servo home position [pulsewidth, units = microseconds (1000-2000)]
    int m_position_exhale_end = -1; //< servo position at the end of exhalation [units = microseconds]
    int m_position_inhale_end = -1; //< servo position at the end of inhalation  [units = microseconds]

    int m_feedback_home = -1; //< servo home position [units = counts (0-1023)]
    int m_feedback_exhale_end = -1; //< servo position at the end of exhalation [units = counts]
    int m_feedback_inhale_end = -1; //< servo position at the end of inhalation  [units = counts]

    int m_tolerance = 4; //< the maximum allowable error in feedback measurement [units = counts]

    // checks to see if the motor is at the target position
    bool checkTarget(int target) {
      if(abs(analogRead(M_FEEDBACK_PIN) - target) > m_tolerance) {
        return false;
      }
      else {
        return true;
      }
    }
};

#endif // VENT_SERVO_H
