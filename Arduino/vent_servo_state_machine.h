#ifndef VENT_SERVO_H
#define VENT_SERVO_H

#include <LabThings.h>
#include <Servo.h>

// state ids:
// 0: home
// 1: exhale end
// 2: inhaling
// 3: inhale end
// 4: exhaling

struct State {
    State* next_state;
    uint32_t duration;
    uint32_t start_time;
    
    State(uint32_t duration = 0) : 
    duration(duration)
    {}
    void exit() {
      
    }
    void enter(uint32_t now) {
      start_time = now;
    }
    uint32_t elapsed(uint32_t now) {
      return now - start_time;
    }
};

class StateMachine {
    uint8_t m_state_count = 0;
    State* m_current_state;
    State* m_states[8];

  public:
    bool addState(State *state) {
      if(m_state_count < 8) {
        m_states[m_state_count] = state;
        ++m_state_count;
        return true;
      }
      else {
        return false;
      }
    }
    
    const State* currentState() const {
      return m_current_state;
    }

    void transition(uint32_t &now) {
      m_current_state->exit();
      m_current_state = m_current_state->next_state;
      m_current_state->enter(now);
    }
    
    void setInitialState(State *state, uint32_t &now) {
      m_current_state = state;
      m_current_state->enter(now);
    }
};

/*!
*/
class SVentServo : public LT_Device {

    //< keep track of servo state
    //enum {
    //  Position_Home = 0,
   //   Position_E_Stop = 1,
    //  Position_I_Stop = 2 //  include transitions?
    //};

    StateMachine m_state_machine;

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
    int m_feedback_home = -1; //< servo home position [units = counts (0-1023)]
    int m_feedback_exhale_end = -1; //< servo position at the end of exhalation [units = counts]
    int m_feedback_inhale_end = -1; //< servo position at the end of inhalation  [units = counts]

    //< the maximum allowable error in feedback measurement [units = counts]
    int m_tolerance = 4;

    int m_current_feedback = 0;
    
    State m_state_home;
    State m_state_exhale;
    State m_state_exhale_end;
    State m_state_inhale;
    State m_state_inhale_end;
    
    //State m_last_state; //= Position_Home;
    //State m_current_state;// = Position_Home;

    //< the time in microseconds that the current state started
    uint32_t m_state_start;

    //< the time in microseconds that the current state is scheduled to end
    uint32_t m_state_end;



    uint32_t t_start = 0;
    uint32_t m_action_period = 2000000;
    int m_current_position = 0;
    //int m_current_feedback;
    int m_current_target;
    int m_state;
    int m_cp_fb;


    /*! check if the motor is at the target position
       returns true if (measured position - target position) < tolerance
    */
    bool checkTarget(int target) {
      m_current_feedback = analogRead(M_FEEDBACK_PIN);
      if (abs(m_current_feedback - target) > m_tolerance) {
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

      m_state_home = State(0);
      m_state_exhale_end = State(1000000);
      m_state_inhale = State(1000000);
      m_state_inhale_end = State(1000000);
      m_state_exhale = State(1000000);
      
      m_state_machine.addState(&m_state_home);
      m_state_machine.addState(&m_state_exhale_end);
      m_state_machine.addState(&m_state_inhale);
      m_state_machine.addState(&m_state_inhale_end);
      m_state_machine.addState(&m_state_exhale);
      
      m_state_machine.setInitialState(&m_state_home, LT_current_time_us);
    }

    void update() {
      
      if(m_running) {
        uint32_t dt = m_state_machine.currentState()->elapsed(LT_current_time_us);
        if(dt >= m_state_machine.currentState()->duration) {
          m_state_machine.transition(LT_current_time_us);
        }
      }
      else {
        // do nothing?
      }
    }

    /*void update() {
      uint32_t dt = LT_current_time_us - t_start;
      if (m_running) {
        
        if (m_current_position == 0) {
          Serial.println("p1");
          setPulseWidth(2000);
          m_current_position = 1;
        }
        else {
          if (m_current_position == 1 && (dt > m_action_period)) {
            setPulseWidth(m_position_inhale_end);
            Serial.println("p2");
            m_current_position = 2;
            t_start = LT_current_time_us;
          }
          else if (m_current_position == 2 && (dt > m_action_period)) {
            setPulseWidth(m_position_exhale_end);
            m_current_position = 1;
            Serial.println("p1");
            t_start = LT_current_time_us;
          }
        }
      }
      else {
        // do nothing if the motor has been stopped
        m_current_position = 0;
      }
      //int p = analogRead(M_FEEDBACK_PIN);
      //Serial.println(p);
    }*/
    // breath/min
    // 1breath/min *1min/60sec * 1000000us/1sec = breath / us
    void setBreathRate(float rate) {
      //m_breath_rate = rate;
      m_action_period = 1.0 / rate * 30000000.0;
      Serial.println(m_action_period);
    }

    void setRunning(bool is_running) {
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
    void calibrateHome(int value) {
      m_position_home = value;
      m_servo.writeMicroseconds(value);
      m_feedback_home = analogRead(M_FEEDBACK_PIN);
    }

    // saves the current position in counts as the servo exhale stop
    void calibrateExhaleEnd(int value) {
      m_position_exhale_end = value;
      m_servo.writeMicroseconds(value);
      m_feedback_exhale_end = analogRead(M_FEEDBACK_PIN);
    }

    // saves the current position in counts as the servo inhale stop
    void calibrateInhaleEnd(int value) {
      m_position_inhale_end = value;
      m_servo.writeMicroseconds(value);
      m_feedback_inhale_end = analogRead(M_FEEDBACK_PIN);
    }
};

#endif // VENT_SERVO_H
