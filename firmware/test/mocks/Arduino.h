// Minimal stand-in for the Arduino core, used only to syntax-check the sketch
// on a PC (make -C firmware/test sketch-check).  Not used on the real board.
#pragma once
#include <cstdlib>
#include <cstring>
#include <string>

#define F(x) x

class String {
 public:
  String(const char* s = "") : s_(s) {}
  bool operator==(const char* o) const { return s_ == o; }
  unsigned length() const { return s_.size(); }
  void trim() {}
  void toCharArray(char* out, unsigned n) const { std::strncpy(out, s_.c_str(), n - 1); out[n - 1] = 0; }
 private:
  std::string s_;
};

struct SerialMock {
  void begin(long) {}
  int available() { return 0; }
  String readStringUntil(char) { return String(); }
  template <typename T> void print(T) {}
  template <typename T> void print(T, int) {}
  template <typename T> void println(T) {}
};
extern SerialMock Serial;
inline void delay(unsigned long) {}
