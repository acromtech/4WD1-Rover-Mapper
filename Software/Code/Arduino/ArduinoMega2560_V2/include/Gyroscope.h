#ifndef Gyroscope_h
#define Gyroscope_h

#include <Arduino.h> 
#include <Wire.h>
#include <MPU6050_light.h>

//MEGA : 20(SDA) 21(SCL) - https://www.arduino.cc/reference/en/language/functions/communication/wire/

class Gyroscope{
  private :
    MPU6050 mpu;
    unsigned long timer = 0;
  public :
    Gyroscope();
    void init();
    void calibration();
    void read();
    void printData();
    double getZDeg();
};
#endif
