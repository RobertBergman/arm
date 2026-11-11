// Signatures copied from the Adafruit PWM Servo Driver Library public API.
#pragma once
#include <cstdint>
class Adafruit_PWMServoDriver {
 public:
  explicit Adafruit_PWMServoDriver(uint8_t) {}
  bool begin(uint8_t prescale = 0) { (void)prescale; return true; }
  void setOscillatorFrequency(uint32_t) {}
  void setPWMFreq(float) {}
  void writeMicroseconds(uint8_t, uint16_t) {}
  uint8_t setPWM(uint8_t, uint16_t, uint16_t) { return 0; }
};
