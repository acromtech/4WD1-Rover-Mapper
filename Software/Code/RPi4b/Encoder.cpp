#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <functional>
#include "Encoder.h"

using namespace std;

Encoder::Encoder(int pinAIn,int pinBIn) {
    pinA = pinAIn;
    pinB = pinBIn;
    pos = 0;
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
}
