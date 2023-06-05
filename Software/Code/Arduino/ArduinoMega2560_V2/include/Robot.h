#ifndef Robot_h
#define Robot_h

#include <Arduino.h> 
#include "Ultrasonic.h"
#include "Gyroscope.h"
#include "Joystick.h"
#include "Motor.h"
#include "Encoder.h"
#include "SimplePID.h"

#define MANUAL_MODE 1 //0->AUTOMATIC 1->MANUAL

#define DEBUG_BAURATE 9600
#define BLUETOOTH_BAURATE 9600

#define FRONT_ULTRASONIC_TRIGGER_PIN 4
#define FRONT_ULTRASONIC_ECHO_PIN 5
#define BACK_ULTRASONIC_TRIGGER_PIN 6
#define BACK_ULTRASONIC_ECHO_PIN 7
#define RIGHT_ULTRASONIC_TRIGGER_PIN 8
#define RIGHT_ULTRASONIC_ECHO_PIN 9

#define LEFT_MOTOR_PIN 2
#define RIGHT_MOTOR_PIN 3

#define LEFT_ENCODER_PINA 18
#define LEFT_ENCODER_PINB 19
#define RIGHT_ENCODER_PINA 20
#define RIGHT_ENCODER_PINB 21

#define PID_PARAMETER_KP 1
#define PID_PARAMETER_KD 0.27
#define PID_PARAMETER_KI 0
#define PID_PARAMETER_UMAX 255

#define WALL_DISTANCE_MARGIN 4  //cm
#define COMPENSATION_ANGLE 0.5  //cm
#define SPEED_MARGIN 50         //°
#define FRONT_SECU_DISTANCE 35  //cm
#define SECU_DISTANCE 25        //cm
#define MAX_SPEED 165           //0-255 (PWM)

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4

class Robot{
  private :
    Ultrasonic usF;
    Ultrasonic usB;
    Ultrasonic usR;
    Gyroscope gyro;
    Joystick joystick;
    Motor motorL;
    Motor motorR;
    Encoder encoderL;
    Encoder encoderR;
    SimplePID pidL;
    SimplePID pidR;

    bool Of,Ob,Or;     //Obstacles
    int initState,CurrentState,NextState;
    bool F,B,L,R,S,SR,SL; //Actuators actions
    double robotOrientation;
    
  public :
    Robot();
    void init();
    void manualProcedure();
    void automaticProcedure();

    void Debug_AllSensor();
    void Debug_AutomaticMode();
};
#endif
