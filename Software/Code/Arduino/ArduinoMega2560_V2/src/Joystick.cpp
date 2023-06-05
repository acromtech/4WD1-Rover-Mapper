#include <Arduino.h>
#include "Joystick.h"

Joystick::Joystick(){}

void Joystick::init(int joyOffsetIn, int switIn){
  joyOffset=joyOffsetIn;
  swit=switIn;
}

float Joystick::joyRawToPhysY(int raw) { /* function joyRawToPhys */////Joystick conversion rule
  float phys = map(raw, MIN_VAL, MAX_VAL, -100 + joyOffset, 100 + joyOffset) - joyOffset;
  phys=phys*(-1);
  return phys;
}

float Joystick::joyRawToPhysX(int raw) { /* function joyRawToPhys */////Joystick conversion rule
  float phys = map(raw, MIN_VAL, MAX_VAL, -100 + joyOffset, 100 + joyOffset) - joyOffset;
  return phys;
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
    angle=(angle*360)/(2*PI);
  }
  return angle;
}

void Joystick::handle_joystick(bool new_val){
  if (new_val == false) return;
  if (new_val == true){
    V = vitesse(new_x_joystick,new_y_joystick);
    Serial.print("Vitesse : ");Serial.println(V);
    Serial.print("Angle : ");
    resultatAngle=AlKashi(new_x_joystick,new_y_joystick);
    if(new_x_joystick<0)resultatAngle=-resultatAngle;
  }
}

void Joystick::read(){
  if (Serial3.available()) {
    val = Serial3.read();
    if(swit==0){
      new_x_joystick=joyRawToPhysX(val);
      //Serial.print("X : ");
      //Serial.println(new_x_joystick);
      swit=1; 
    }
    else{
      new_y_joystick=joyRawToPhysY(val);
      //Serial.print("Y : ");
      //Serial.println(new_y_joystick);
      swit=0;
    }
    handle_joystick(true); /*handle_joystick((new_x_joystick != old_x_joystick) || (new_y_joystick != old_y_joystick));*/
   // old_x_joystick = new_x_joystick;
   //old_y_joystick = new_y_joystick;
  }
}

double Joystick::getAngleToSet(){ return resultatAngle; }

float Joystick::getSpeedToSet(){ return V; }
