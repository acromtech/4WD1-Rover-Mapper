#include <Arduino.h>
#include "Encoder.h"

// Initialize the static member variable
Encoder* Encoder::instance = nullptr;

Encoder::Encoder() {}

void Encoder::init(int pinAIn, int pinBIn) {
  pinA = pinAIn;
  pinB = pinBIn;
  pos = 0;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA),staticIT_readEncoder,RISING);
}

void Encoder::printDebugEncoder() {
  int a = digitalRead(pinA);
  int b = digitalRead(pinB);
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
  Serial.println();
}

int Encoder::getEncoderPos() { return pos; }

void Encoder::staticIT_readEncoder() {
  if (instance) {
    if (digitalRead(instance->pinB) > 0) instance->pos++;
    else instance->pos--;
  }
}
