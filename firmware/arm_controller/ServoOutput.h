// Abstraction of "something that can send a pulse width to a servo channel".
// The motion code depends on this interface only, so the PCA9685 can be
// swapped for another driver (or a test recorder) without touching it.
#pragma once

class ServoOutput {
 public:
  virtual ~ServoOutput() {}
  virtual void begin() = 0;
  virtual void writeMicroseconds(unsigned char channel, unsigned int microseconds) = 0;
  virtual void release(unsigned char channel) = 0;  // stop driving (servo goes limp)
};
