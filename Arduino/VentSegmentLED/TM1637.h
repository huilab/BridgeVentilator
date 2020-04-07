#ifndef __TM1637_H__
#define __TM1637_H__

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80


// segment bit order:
//      0
//     ---
//  5 |   | 1
//     -6-
//  4 |   | 2
//     ---
//      3
// the dots are controlled by bit 8 (MSB) on segment 2
// the 8th bit is only used on the second segment

uint8_t segmap[] {
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8, B
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001,     // F
  0b01110110,     // X
  0b00110000,     // I
  0b01000000      // -
};


class TM1637 {

  public:
    TM1637(uint8_t clk, uint8_t dio, uint32_t bit_delay = 100)
      : m_clk(clk), m_dio(dio), m_bit_delay(bit_delay)
    {

    }

    void begin()
    {
      pinMode(m_clk, INPUT_PULLUP);
      pinMode(m_dio, INPUT_PULLUP);
      digitalWrite(m_clk, LOW);
      digitalWrite(m_dio, LOW);
    }

    void setBrightness(uint8_t brightness, bool on = true)
    {
      m_brightness = (brightness & 0x7) | (on ? 0x08 : 0x00);
    }

    void showNumber(int num,  bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0, uint8_t base = 10)
    {
      uint8_t digits[4];
      
      bool is_negative = (num < 0) ? true : false;
      if (num == 0) {
        for(uint8_t i = 0; i < (length-1); i++) {
          digits[i] = 0;
        }
        digits[length-1] = segmap[0];
      }
      else {
        
        for (int i = length - 1; i >= 0; --i)
        {
          uint8_t digit = num % base;

          if (digit == 0 && num == 0 && leading_zero == false)
            // Leading zero is blank
            digits[i] = 0;
          else
            digits[i] = segmap[digit];

          if (digit == 0 && num == 0 && is_negative) {
            digits[i] = segmap[18];
            is_negative = false;
          }

          num /= base;
        }
      }
        setSegments(digits, length, pos);
    }
    /*
      For displays with dots between each digit:
      //!        * 0.000 (0b10000000)
      //!        * 00.00 (0b01000000)
      //!        * 000.0 (0b00100000)
      //!        * 0.0.0.0 (0b11100000)
      //!        For displays with just a colon:
      //!        * 00:00 (0b01000000)
      //!        For displays with dots and colons colon:
      //!        * 0.0:0.0 (0b11100000)
    */
    void setSegments(const uint8_t segments[], uint8_t length = 4, uint8_t pos = 0)
    {
      // Write COMM1
      start();
      writeByte(TM1637_I2C_COMM1);
      stop();

      // Write COMM2 + first digit address
      start();
      writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

      // Write the data bytes
      for (uint8_t k = 0; k < length; k++)
        writeByte(segments[k]);

      stop();

      // Write COMM3 + brightness
      start();
      writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
      stop();
    }

    void clear()
    {
      uint8_t data[] = { 0, 0, 0, 0 };
      setSegments(data);
    }

    void start()
    {
      pinMode(m_dio, OUTPUT);
      bitDelay();
    }

    void stop()
    {
      pinMode(m_dio, OUTPUT);
      bitDelay();
      pinMode(m_clk, INPUT);
      bitDelay();
      pinMode(m_dio, INPUT);
      bitDelay();
    }

    bool writeByte(uint8_t b)
    {
      uint8_t data = b;

      // 8 Data Bits
      for (uint8_t i = 0; i < 8; i++) {
        // CLK low
        pinMode(m_clk, OUTPUT);
        bitDelay();

        // Set data bit
        if (data & 0x01)
          pinMode(m_dio, INPUT);
        else
          pinMode(m_dio, OUTPUT);

        bitDelay();

        // CLK high
        pinMode(m_clk, INPUT);
        bitDelay();
        data = data >> 1;
      }

      // Wait for acknowledge
      // CLK to zero
      pinMode(m_clk, OUTPUT);
      pinMode(m_dio, INPUT);
      bitDelay();

      // CLK to high
      pinMode(m_clk, INPUT);
      bitDelay();
      uint8_t ack = digitalRead(m_dio);
      if (ack == 0)
        pinMode(m_dio, OUTPUT);


      bitDelay();
      pinMode(m_clk, OUTPUT);
      bitDelay();

      return ack;
    }

    void bitDelay()
    {
      delayMicroseconds(m_bit_delay);
    }

  private:
    uint8_t m_clk;
    uint8_t m_dio;
    uint32_t m_bit_delay; //< delay between bits in [us]
    uint8_t m_brightness;
};


#endif // __TM1637_H__
