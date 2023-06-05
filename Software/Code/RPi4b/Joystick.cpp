#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "Joystick.h"

Joystick::Joystick(const char* portName, int baudRate){
  swit=0;
  fd=serialOpen(portName,baudRate);
}

Joystick::~Joystick() {
    if (fd != -1) serialClose(fd);
}

float Joystick::joyRawToPhysY(int raw) {
  return -customMap(raw, MIN_VAL, MAX_VAL, -100 + JOYOFFSET, 100 + JOYOFFSET) - JOYOFFSET;
}

float Joystick::joyRawToPhysX(int raw) {
  return customMap(raw, MIN_VAL, MAX_VAL, -100 + JOYOFFSET, 100 + JOYOFFSET) - JOYOFFSET;
}

float Joystick::vitesse(int x, int y){
  float v = sqrt(pow(x,2)+pow(y,2));
  if(v>100) v=100;
  else if(x<5 && x>-5 && y<5 && y>-5) v=0;
  return v;
}

double Joystick::AlKashi(int x, int y){
  if(x==0){
    if(y<0)angle=180;
    else angle=0;
  }
  else if(y==0){
    if(x<0)angle=-90;
    else angle=90;
  }
  else if(x<5 && x>-5 && y<5 && y>-5)angle=0;
  else{
    double hypotenus=sqrt(pow(x,2)+pow(y,2));
    angle=acos((pow(x,2)-pow(y,2)-pow(hypotenus,2))/((-2)*(hypotenus)*(y)));
    angle=(angle*360)/(2*M_PI);
  }
  return angle;
}

void Joystick::handle_joystick(bool new_val){
  if (new_val == false) return;
  if (new_val == true){
    V = vitesse(new_x_joystick,new_y_joystick);
    resultatAngle=AlKashi(new_x_joystick,new_y_joystick);
    if(new_x_joystick<0)resultatAngle=-resultatAngle;
  }
}

void Joystick::read(){
  if (serialDataAvail(fd)) {
    val = (uint8_t)serialGetchar(fd);
    if(swit==0){
      new_x_joystick=joyRawToPhysX(val);
      swit=1; 
    }
    else{
      new_y_joystick=joyRawToPhysY(val);
      swit=0;
    }
    handle_joystick(true);
  }
}

double Joystick::getAngleToSet(){ return resultatAngle; }
float Joystick::getSpeedToSet(){ return V; }

float Joystick::customMap(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    return ((value - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
}
