#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include "Motor.h"

using namespace std;

Motor::Motor(unsigned int PWMpin) {
  pin=PWMpin;
  pinMode(pin,PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);  // Mode Mark:Space
  pwmSetRange(2000);  // Set the PWM range to match the servo range (1000-2000)
  pwmSetClock(192);   // Set the PWM clock to 50 Hz (typical for servos)
}

void Motor::setMotor(int dir,int pwr){
  if(dir==FORWARD) pwmWrite(pin,PWR_MAX_FOREWARD);
  else if(dir==(BACKWARD)) pwmWrite(pin,PWR_MAX_BACKWARD);
  else pwmWrite(pin,PWR_STOP);
}

void Motor::testMotor(){
  cout<<"FORWARD"<<endl;
  setMotor(FORWARD,255);
  delay(2000);
  cout<<"BACKWARD"<<endl;
  setMotor(BACKWARD,255);
  delay(2000);
  cout<<"STOP"<<endl;
  setMotor(STOP,0);
  delay(2000);
}

int Motor::getMotorPin(){ return pin; }
