/*!
 * 
 * 
 */

#ifndef VENT_SERVO_H
#define VENT_SERVO_H

#include <LabThings.h>
//#include <Servo.h>
#include <ESP32Servo.h>


// state ids:
// 0: home
// 1: exhale end
// 2: inhaling
// 3: inhale end
// 4: exhaling
enum ServoState {
  State_Home = 0,
  State_ExEnd = 1,
  State_Inhale = 2,
  State_InEnd = 3,
  State_Exhale = 4
};

/*!
*/
class VentServo : public LT_Device {

    // a servo object
    Servo m_servo;

    //< keep track if the machine is running
    bool m_running = false;

    //< wiring
    const int M_SERVO_PIN; //< the pin the servo is attached to
    const int M_FEEDBACK_PIN; //< the pin the servo is attached to

    //< the servo pulsewidth for each state in microseconds [1000, 2000]
    int m_position_home = 2000; //< servo home position [pulsewidth, units = microseconds (1000-2000)]
    int m_position_exhale_end = 1750; //< servo position at the end of exhalation [units = microseconds]
    int m_position_inhale_end = 1000; //< servo position at the end of inhalation  [units = microseconds]

    //< the feedback data for each state in counts [0-1023]
    int m_feedback_home = 157; //< servo home position [units = counts (0-1023)]
    int m_feedback_exhale_end = 203; //< servo position at the end of exhalation [units = counts]
    int m_feedback_inhale_end = 345; //< servo position at the end of inhalation  [units = counts]

    //< the maximum allowable error in feedback measurement [units = counts]
    int m_tolerance = 8; // 1 count is about 2 degrees

    //< the current feedback measurement in counts [0-1023]
    int m_current_feedback = 0;
    //< the target feedback position at the end of this state in counts [0-1023]
    int m_target_feedback = 0;
    //< the feedback target from the previous target in counts [0-1023]
    int m_target_feedback_last = 0;

    
    //< the servo pulse width target for the end of this state in us [1000-2000]
    int m_target_position = 0;
    //< the servo pulse width target from the previous state in us [1000-2000]
    int m_target_position_last = 0;

    //< the current state
    ServoState m_current_state = State_Home;
    //< the time in microseconds that the current state started
    uint32_t m_state_start;
    //< the time in microseconds that the current state is scheduled to last
    uint32_t m_state_duration;

    float m_volume = 800.0; //< the tidal volume. a full stroke is 800 cc
    

    uint32_t m_inhale_period = 1000000; //< period for inhale in us
    uint32_t m_exhale_period = 1000000; //< period for exhale in us
    uint32_t m_breath_period = 4000000; //< period for one breath in us

    /*! check if the motor is at the target position
       returns true if (measured position - target position) < tolerance
    */
    bool checkTarget(int target) {
      m_current_feedback = analogRead(M_FEEDBACK_PIN);
        Serial.print("Read: ");
        Serial.print(m_current_feedback);
        Serial.print(" Expected: ");
        Serial.println(target);
      if (abs(m_current_feedback - target) > m_tolerance) {
        Serial.println("Position error");
        return false;
      }
      else {
        return true;
      }
    }

  public:
    VentServo(const uint8_t id, const uint8_t servo_pin, const uint8_t feedback_pin) : LT_Device(id),
      M_SERVO_PIN(servo_pin), M_FEEDBACK_PIN(feedback_pin) {}

    void begin() {
      pinMode(M_SERVO_PIN, OUTPUT);
      m_servo.attach(M_SERVO_PIN, 1000, 2000);
      pinMode(M_FEEDBACK_PIN, INPUT);
      Serial.print("servo start pin");
      Serial.println(M_SERVO_PIN);
    }

    void update() {

      if (m_running) {
        uint32_t dt = LT_current_time_us - m_state_start;
        if(dt < 10000) { // at max check every 10 millisecond
          return;
        }
        int32_t last_duration = m_state_duration;
        if (dt > m_state_duration) {
          // time to change states
          switch (m_current_state) {
          case State_Home: {
            // start a new breath cycle
            Serial.println("Starting");
            m_current_state = State_ExEnd;
            m_state_duration = 1000000; // start exhaling immediately
            m_target_position_last = m_position_exhale_end;
            m_target_feedback_last = m_feedback_exhale_end;
            m_target_position = m_position_exhale_end;
            m_target_feedback = m_feedback_exhale_end;
            break;
          }
          case State_ExEnd: {
            // start inhaling
            Serial.println("Inhale");
            checkTarget(m_target_feedback_last);
            m_current_state = State_Inhale;
            m_state_duration = m_inhale_period;
            m_target_position_last = m_position_exhale_end;
            m_target_feedback_last = m_feedback_exhale_end;
            m_target_position = m_position_inhale_end;
            m_target_feedback = m_feedback_inhale_end;
            break;
          }
          case State_Inhale: {
            // end inhalation
            Serial.println("Paused");
            m_current_state = State_InEnd;
            m_state_duration = (m_breath_period - m_inhale_period - m_exhale_period) >> 1; //< half
            m_target_position_last = m_position_inhale_end;
            m_target_feedback_last = m_feedback_inhale_end;
            m_target_position = m_position_inhale_end;
            m_target_feedback = m_feedback_inhale_end;
            break;
          }
          case State_InEnd: {
            // start exhaling
            Serial.println("Exhale");
            checkTarget(m_target_feedback_last);
            m_current_state = State_Exhale;
            m_state_duration = m_exhale_period;
            m_target_position_last = m_position_inhale_end;
            m_target_feedback_last = m_feedback_inhale_end;
            m_target_position = m_position_exhale_end;
            m_target_feedback = m_feedback_exhale_end;
            break;
          }
          case State_Exhale: {
            // end exhalation
            Serial.println("Pause");
            m_current_state = State_ExEnd;
            m_state_duration = (m_breath_period - m_inhale_period - m_exhale_period) >> 1; //< half
            m_target_position_last = m_position_exhale_end;
            m_target_feedback_last = m_feedback_exhale_end;
            m_target_position = m_position_exhale_end;
            m_target_feedback = m_feedback_exhale_end;
            break;
          }
        }
        m_state_start += last_duration;
      }
      else {
        // this should run every few milliseconds
        // interpolate the next target position
        int pw = ( ( (int32_t)dt * (m_target_position - m_target_position_last) ) / (int32_t)m_state_duration) + m_target_position_last;
        setPulseWidth(pw);
        /*
        Serial.print("dt: ");
        Serial.print(dt/1000);
        Serial.print("\tm_state_duration: ");
        Serial.print(m_state_duration/1000);
        
        Serial.print("\tm_target_position ");
        Serial.print(m_target_position/1000);
        
        Serial.print("\tm_target_position_last ");
        Serial.print(m_target_position_last/1000);
        Serial.print("\tset pw: ");
        Serial.println(pw);
        
        // check position based on last interpolataed target
        // current position should be dt/duration * (target-last)
        uint32_t delay_start = micros();
        int expected_feedback = (  ( (int32_t)dt * (m_target_feedback - m_target_feedback_last) ) / (int32_t)m_state_duration) + m_target_feedback_last;
        while(!checkTarget(expected_feedback)){
          //
        }
        uint32_t total_delay = micros() -delay_start;
        Serial.print("total delay:" );
        Serial.println(total_delay);*/
       /* static int last_expected_feedback = 0;
        if(last_expected_feedback) {
          checkTarget(last_expected_feedback);
        }
        
        last_expected_feedback = (  ( (int32_t)dt * (m_target_feedback - m_target_feedback_last) ) / (int32_t)m_state_duration) + m_target_feedback_last;*/
      }
    }
    else {
      // not running, do nothing
    }
  } //update
  
  // breath/min
  // 1breath/min *1min/60sec * 1000000us/1sec = breath / us
  void setBreathRate(const float breath_rate) {
      m_breath_period = 1.0 / breath_rate * 60000000.0;
      Serial.print("Set breath period: ");
      Serial.println(m_breath_period);
    }

    void setVolume(const float volume) {
      
    }

    void setInspiratoryPeriod(const float ip) {
      m_inhale_period = ip * 1000000;
    }

    void setExpiratoryPeriod(const float xp) {
      m_exhale_period = xp * 1000000;
    }

    void setRunning(const bool is_running) {
      m_running = is_running;
    }

    // instructs the servo to move to the angle
    // angle: the position in degrees
    void goToAngle(const int angle) {
      // this assumes the motor has a 180 range
      int pw = map(angle, 0, 180, 1000, 2000);
      setPulseWidth(pw);
    }


    void setPulseWidth(const int pw) {
      m_servo.writeMicroseconds(pw);
    }

    void goHome() {
      m_servo.writeMicroseconds(m_position_home);
    }

    int getSetPosition() {
      return m_servo.readMicroseconds();
    }

    int getFeedbackPosition() {
      return analogRead(M_FEEDBACK_PIN);
    }

    // the current position in counts is saved as the
    void calibrateHome(const int value) {
      m_position_home = value;
      m_servo.writeMicroseconds(value);
      m_feedback_home = analogRead(M_FEEDBACK_PIN);
      Serial.print("set postion feedback: ");
      Serial.println(m_feedback_home);
    }

    // saves the current position in counts as the servo exhale stop
    void calibrateExhaleEnd(const int value) {
      m_position_exhale_end = value;
      m_servo.writeMicroseconds(value);
      m_feedback_exhale_end = analogRead(M_FEEDBACK_PIN);
      Serial.print("set postion feedback: ");
      Serial.println(m_feedback_exhale_end);
    }

    // saves the current position in counts as the servo inhale stop
    void calibrateInhaleEnd(const int value) {
      m_position_inhale_end = value;
      m_servo.writeMicroseconds(value);
      m_feedback_inhale_end = analogRead(M_FEEDBACK_PIN);
      Serial.print("set postion feedback: ");
      Serial.println(m_feedback_inhale_end);
    }
};

#endif // VENT_SERVO_H
