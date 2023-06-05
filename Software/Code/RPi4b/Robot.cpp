#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include "Ultrasonic.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
//#include "SimplePID.h"
#include "Robot.h"

using namespace std;

// Variables statiques pour les compteurs d'encodeur
static int encoderPosL = 0;
static int encoderPosR = 0;

void staticIT_readEncoderL() {
  int b = digitalRead(LEFT_ENCODER_PINB);
  if (b > 0)
    encoderPosL++;
  else
    encoderPosL--;
}

void staticIT_readEncoderR() {
  int b = digitalRead(RIGHT_ENCODER_PINB);
  if (b > 0)
    encoderPosR++;
  else
    encoderPosR--;
}

Robot::Robot() :  gyro(0x68),
                  motorL(LEFT_MOTOR_PIN),
                  motorR(RIGHT_MOTOR_PIN),
                  usF(FRONT_ULTRASONIC_TRIGGER_PIN,FRONT_ULTRASONIC_ECHO_PIN,30000),
                  usB(BACK_ULTRASONIC_TRIGGER_PIN,BACK_ULTRASONIC_ECHO_PIN,30000), 
                  usR(RIGHT_ULTRASONIC_TRIGGER_PIN,RIGHT_ULTRASONIC_ECHO_PIN,30000),
                  //joystick("/dev/serial0", 9600)
                  encoderL(LEFT_ENCODER_PINA,LEFT_ENCODER_PINB),        //Green(interrupt pin),Yellow
                  encoderR(RIGHT_ENCODER_PINA,RIGHT_ENCODER_PINB)       //Green(interrupt pin),Yellow
                  {
  wiringPiISR(LEFT_ENCODER_PINA, INT_EDGE_RISING, staticIT_readEncoderL);
  wiringPiISR(RIGHT_ENCODER_PINA, INT_EDGE_RISING, staticIT_readEncoderR);
  //pidL.setParams(PID_PARAMETER_KP,PID_PARAMETER_KD,PID_PARAMETER_KI,PID_PARAMETER_UMAX);
  //pidR.setParams(PID_PARAMETER_KP,PID_PARAMETER_KD,PID_PARAMETER_KI,PID_PARAMETER_UMAX);
}

void Robot::goForward(){
  motorL.setMotor(FORWARD,MAX_SPEED);
  motorR.setMotor(BACKWARD,MAX_SPEED);
}
void Robot::goBackward(){
  motorL.setMotor(BACKWARD,MAX_SPEED);
  motorR.setMotor(FORWARD,MAX_SPEED);
}
void Robot::turnLeft(){
  motorL.setMotor(BACKWARD,MAX_SPEED);
  motorR.setMotor(BACKWARD,MAX_SPEED);
}
void Robot::turnRight(){
  motorL.setMotor(FORWARD,MAX_SPEED);
  motorR.setMotor(FORWARD,MAX_SPEED);
}
void Robot::stop(){
  motorL.setMotor(STOP,0); 
  motorR.setMotor(STOP,0);
}

void Robot::manualProcedure(){
  /****************************/
  /* Read inputs              */
  /****************************/
  //joystick.read();

  usF.read();
  usB.read();
  usR.read();

  if (usF.getDistance()<=SECU_DISTANCE)    Of=1; else Of=0;
  if (usB.getDistance()<=SECU_DISTANCE)    Ob=1; else Ob=0;
  if (usR.getDistance()<=SECU_DISTANCE)    Or=1; else Or=0;

  /****************************/
  /* Write outputs            */
  /****************************/
  /*
  if(joystick.getSpeedToSet()==0){
    motorL.setMotor(STOP,0);
    motorR.setMotor(STOP,0);
  }
  else if(joystick.getAngleToSet()>=-45 && joystick.getAngleToSet()<45 && !Of){  //FORWARD
    motorL.setMotor(FORWARD,joystick.getSpeedToSet()*2.5);
    motorR.setMotor(BACKWARD,joystick.getSpeedToSet()*2.5);
  }
  else if(joystick.getAngleToSet()>=45 && joystick.getAngleToSet()<135){  //LEFT
    motorL.setMotor(FORWARD,joystick.getSpeedToSet()*2.5);
    motorR.setMotor(FORWARD,joystick.getSpeedToSet()*2.5);
  }
  else if(joystick.getAngleToSet()>=-135 && joystick.getAngleToSet()<-45){  //RIGHT
    motorL.setMotor(BACKWARD,joystick.getSpeedToSet()*2.5);
    motorR.setMotor(BACKWARD,joystick.getSpeedToSet()*2.5);
  }
  else if(((joystick.getAngleToSet()>=-180 && joystick.getAngleToSet()<-135) || (joystick.getAngleToSet()>=135 && joystick.getAngleToSet()<180)) && !Ob){ //BACKWARD
    motorL.setMotor(BACKWARD,joystick.getSpeedToSet()*2.5);
    motorR.setMotor(FORWARD,joystick.getSpeedToSet()*2.5);
  }
  */
}

void Robot::automaticProcedure(){
    usF.read();
    usB.read();
    usR.read();
    
    if (usF.getDistance()<=FRONT_SECU_DISTANCE)   Of=1; else Of=0;
    if (usB.getDistance()<=SECU_DISTANCE)         Ob=1; else Ob=0;
    if (usR.getDistance()<=SECU_DISTANCE)         Or=1; else Or=0;

    /************/
    /* F bloc  */
    /************/
    switch(CurrentState){
        case 0 :  if(Or && Ob)                          NextState = 1;   break;
        case 1 :  if(!Of)                               NextState = 2;   break;
        case 2 :  if(Of){ //TURN LEFT
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    NextState = 3;
                    robotOrientation=(gyro.getYawAngle())+90-SPEED_MARGIN;
                  }
                  else if (!Or){ //TURN RIGHT
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    NextState = 4;
                    robotOrientation=(gyro.getYawAngle())-90+SPEED_MARGIN;
                  }
                  else if (usR.getDistance()<SECU_DISTANCE-WALL_DISTANCE_MARGIN){ //LEFT COMPENSATION
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    robotOrientation=(gyro.getYawAngle())+COMPENSATION_ANGLE;
                    NextState = 5;
                  }
                  else if (usR.getDistance()>SECU_DISTANCE+WALL_DISTANCE_MARGIN){ //RIGHT COMPENSATION
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    robotOrientation=(gyro.getYawAngle())-COMPENSATION_ANGLE;
                    NextState = 6; 
                  }
                  break;
        case 3 :  if(robotOrientation<=gyro.getYawAngle())   NextState = 11;  break;  //LEFT
        case 4 :  if(robotOrientation>=gyro.getYawAngle())   NextState = 11;  break;  //RIGHT
        case 5 :  if(robotOrientation<=gyro.getYawAngle())   NextState = 2;   break;  //LEFT
        case 6 :  if(robotOrientation>=gyro.getYawAngle())   NextState = 2;   break;  //RIGHT
        case 11 : if(Or)                                 NextState = 2;   break;
        default :                                                         break;
        
    }
    
    /************/
    /* M bloc  */
    /************/   
    if(initState){
        CurrentState = 0;
        initState = 0;
    }
    else CurrentState = NextState;
    
    /************/
    /* G bloc   */
    /************/
    B = 0;
    F = (CurrentState == 2) || (CurrentState == 11);
    L = (CurrentState == 3);
    R = (CurrentState == 4);
    S = 0;
    SL = (CurrentState == 5);
    SR = (CurrentState == 6);
    
    /****************************/
    /* Write output     */
    /****************************/
    if(F) goForward();
    else if(B) goBackward();
    else if(L) turnLeft();
    else if(R) turnRight();
    else if(S) stop();
    else if(SL){
      motorL.setMotor(STOP,0);
      motorR.setMotor(BACKWARD,MAX_SPEED);
    }
    else if(SR){
      motorL.setMotor(FORWARD,MAX_SPEED);
      motorR.setMotor(STOP,0);
    }
}

void Robot::Debug_Motor(){
  motorL.testMotor();
  motorR.testMotor();
}

void Robot::Debug_Mobility(){
  cout<<"goForward"<<endl;  goForward();  delay(2000);
  cout<<"goBackward"<<endl; goBackward(); delay(2000);
  cout<<"turnLeft"<<endl;   turnLeft();   delay(2000);
  cout<<"turnRight"<<endl;  turnRight();  delay(2000);
  cout<<"stop"<<endl;       stop();
}

void Robot::Debug_Encoder(){
  cout<<"LeftEnc : "<<encoderPosL<<"\t RightEnc : "<<encoderPosR<<endl;
}

void Robot::Debug_UltrasonicSensors(){
  usF.read(); cout<<" FrontUS : "<<usF.getDistance();
  usB.read(); cout<<"\t BackUS : "<<usB.getDistance();
  usR.read(); cout<<"\t RightUS : "<<usR.getDistance()<<endl;
}

void Robot::Debug_Gyroscope(){
  cout<<" YawGyro : "<<gyro.getYawAngle()<<endl;
}

void Robot::Debug_AllSensor(){
  cout<<"LeftEnc : "<<encoderPosL<<"\t RightEnc : "<<encoderPosR;
  usF.read(); cout<<"\t FrontUS : "<<usF.getDistance();
  usB.read(); cout<<"\t BackUS : "<<usB.getDistance();
  usR.read(); cout<<"\t RightUS : "<<usR.getDistance();
  cout<<"\t YawGyro : "<<gyro.getYawAngle()<<endl;
}

void Robot::Debug_AutomaticMode(){
  cout<<CurrentState<<"\t"<<Of<<"\t"<<Ob<<"\t"<<Or;
}
