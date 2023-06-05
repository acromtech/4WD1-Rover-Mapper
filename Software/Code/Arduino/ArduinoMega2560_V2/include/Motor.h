#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Servo.h>

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4

class Motor{
  private :
    Servo DCmotor;
    int pin;
  public :
    Motor();
    void init(int PWMpin);
    void setMotor(int dir,float pwr);
    void testMotor();
    int getMotorPin();
};
#endif 
