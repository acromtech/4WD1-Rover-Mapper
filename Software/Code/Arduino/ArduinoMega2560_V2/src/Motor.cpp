#include <Arduino.h>
#include <Servo.h>
#include "Motor.h"

Motor::Motor() {}

void Motor::init(int PWMpin){ 
  DCmotor.attach(PWMpin,1000,2000);
}

void Motor::setMotor(int dir,float pwr){
  if(dir==FORWARD) DCmotor.write((pwr*90/255)+90);                //wheel FORWARD
  else if(dir==(BACKWARD)) DCmotor.write((255-pwr)*90/255);       //wheel BACKWARD
  else DCmotor.write(90);
}

void Motor::testMotor(){
  setMotor(FORWARD,255);
  delay(200);
  setMotor(STOP,0);
  delay(200);
  setMotor(BACKWARD,255);
  delay(200);
  setMotor(STOP,0);
  delay(200);
}

int Motor::getMotorPin(){ return pin; }
