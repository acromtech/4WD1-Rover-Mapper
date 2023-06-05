#include <Arduino.h>
#include <Wire.h>
#include "Gyroscope.h"
#include <MPU6050_light.h>

Gyroscope::Gyroscope() : mpu(Wire){}

void Gyroscope::init(){
  Wire.begin();
  mpu.begin();    
}

void Gyroscope::calibration(){
  mpu.calcGyroOffsets();                          // This does the calibration
}

void Gyroscope::read(){
  mpu.update();
}

void Gyroscope::printData(){
  Serial.println(mpu.getAngleZ());
}

double Gyroscope::getZDeg(){ return (mpu.getAngleZ()); }
