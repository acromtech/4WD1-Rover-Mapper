#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder {
private:
  int pos;
  int pinA, pinB;
  static Encoder* instance;
  static void staticIT_readEncoder();

public:
  Encoder();
  void init(int pinAIn, int pinBIn);
  int getEncoderPos();
  void printDebugEncoder();
};

#endif
