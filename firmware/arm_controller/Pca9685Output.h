// ServoOutput implemented with a PCA9685 16-channel PWM board over I2C.
#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "ServoOutput.h"

class Pca9685Output : public ServoOutput {
 public:
  explicit Pca9685Output(unsigned char address = 0x40) : driver_(address) {}

  void begin() override {
    Wire.begin();
    driver_.begin();
    driver_.setOscillatorFrequency(27000000);
    driver_.setPWMFreq(50);  // standard hobby-servo frame rate
  }

  void writeMicroseconds(unsigned char channel, unsigned int microseconds) override {
    driver_.writeMicroseconds(channel, microseconds);
  }

  void release(unsigned char channel) override { driver_.setPWM(channel, 0, 4096); }

 private:
  Adafruit_PWMServoDriver driver_;
};
