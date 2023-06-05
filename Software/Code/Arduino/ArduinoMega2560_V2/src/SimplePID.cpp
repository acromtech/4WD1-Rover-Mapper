#include <Arduino.h>
#include "SimplePID.h"

//Constructeur

void SimplePID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
  kp=kpIn;
  kd=kdIn;
  ki=kiIn;
  umax=umaxIn;
}

void SimplePID::evalu(int value, int target, float deltaT, int &pwr, int &dir){
  int e=target-value;                     //error
  float dedt=(e-eprev)/deltaT;          //derivative
  eintegral=eintegral+e*deltaT;         //integral
  float u=kp*e+kd*dedt+ki*eintegral;    //control signal

  //motor power
  pwr=(int)fabs(u);
  if(pwr>umax)pwr=umax;
  
  //motor direction
  dir=FORWARD;
  if(u<0) dir=BACKWARD;
  
  eprev=e;                              //store previous error
}

void SimplePID::PIDAllWheeltest(){
  /*
  int target[NB_MOTOR];
  if (Serial.available()>0) {            // manual control of wheels via terminal
    char c = Serial.read();
    if (c == 'f'){
      target[LEFT]=3000;
      target[RIGHT]=-3000;
    }
    if (c == 'b'){
      target[LEFT]=-3000;
      target[RIGHT]=3000;
    }
    if (c == 'l'){
      target[LEFT]=-3000;
      target[RIGHT]=-3000;
    }
    if (c == 'r'){
      target[LEFT]=3000;
      target[RIGHT]=3000;
    }
  }
  long currT=micros();
  float deltaT=((float)(currT-prevT))/1.0e6;
  prevT=currT;

  //read the position in an atomic block to avoid a potential misread
  int posi[NB_MOTOR];
  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k=0;k<NB_MOTOR;k++) posi[k]=pos[k];
  //}

  //loop through the motors
  for(int k=0,pwr,dir;k<NB_MOTOR;k++){
    pid[k].evalu(posi[k],target[k],deltaT,pwr,dir);             //evaluate the control signal
    setMotor(motors[k],dir,pwr);                                //signal the motor
  }
  
  for(int k=0;k<NB_MOTOR;k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(posi[k]);
  }
  Serial.println();
  */
}
