#ifndef Robot_h
#define Robot_h

/**-----------------------------------
 * 
 * LYNXMOTION 4WD1 ROBOT PARAMETERS
 * 
 * -----------------------------------
 * 
 * The app -------------
 * To control the bot find in the github repository and install the .apk app in your favorite Android device.
 * If you choose to create your HMI interface powered by raspberyPi on Android OS you can install LineageOS (https://lineageos.org/) thanks to the UNOFFICIAL version by konstakang :
 *      - For the RaspberryPi 3 : https://konstakang.com/devices/rpi3/
 *      - For the RaspberryPi 4 : https://konstakang.com/devices/rpi4/
 */ 

#include <wiringPi.h>
#include "Ultrasonic.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
//#include "SimplePID.h"

#define MANUAL_MODE 1 //0->AUTOMATIC 1->MANUAL

/**
 * WARNING : Use wiringpi pin on your RaspberryPi (more informations on https://pinout.xyz/pinout/wiringpi)
 */
#define FRONT_ULTRASONIC_TRIGGER_PIN 13
#define FRONT_ULTRASONIC_ECHO_PIN 14
#define BACK_ULTRASONIC_TRIGGER_PIN 4
#define BACK_ULTRASONIC_ECHO_PIN 5
#define RIGHT_ULTRASONIC_TRIGGER_PIN 2
#define RIGHT_ULTRASONIC_ECHO_PIN 3

/**
 * WARNING : Use PWM pin on your RaspberryPi (more informations on https://pinout.xyz/pinout/pwm)
 * GPIO 12 & 18 -> PWM0 (WiringPi 1 & 26)
 * GPIO 13 & 19 -> PWM1 (WiringPi 23 & 24)
 * Setup GPIO 18 & 19 to ALT5 mode (PWM) with the following commands (Include in Makefile)
 *  gpio -g mode 18 alt5
 *  gpio -g mode 19 alt5
 */
#define LEFT_MOTOR_PIN 1
#define RIGHT_MOTOR_PIN 24

/**
 * To active ISR for wiringPi on your RaspberryPi follow the steps below :
 * 1. Open a terminal and put the following commands :
 *    sudo apt-get update
 *    sudo apt-get upgrade wiringpi
 * 2. Check free GPIO pin thanks to the folowing command and save the wiringPi number
 *    gpio readall
 * 3. Open the /boot/config.txt file thanks to the following command :
 *    sudo nano /boot/config.txt
 * 4. Put the following lines into the file and replace <GPIO_NUMBER> by your saved GPIO number
 *    dtoverlay=gpio-irq,<GPIO_NUMBER>=active
 * 5. Save modifications and reboot system :
 *    sudo reboot
*/
#define LEFT_ENCODER_PINA 22    //Green - ISR PIN : Check the steps
#define LEFT_ENCODER_PINB 23    //Yellow
#define RIGHT_ENCODER_PINA 10   //Green - ISR PIN : Check the steps
#define RIGHT_ENCODER_PINB 11   //Yellow

/**
 * WARNING : Gyroscope SDA & SCL pins are respectively connected to the GPIO 2 & 3 (WiringPi 8 & 9)
 */

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
    //Privates attributes
    Ultrasonic usF;
    Ultrasonic usB;
    Ultrasonic usR;
    MPU6050 gyro;
    Motor motorL;
    Motor motorR;
    Encoder encoderL;
    Encoder encoderR;
    
    //SimplePID pidL;
    //SimplePID pidR;

    bool Of,Ob,Or;          //Obstacles
    int initState,CurrentState,NextState;
    bool F,B,L,R,S,SR,SL;   //Actuators actions
    double robotOrientation;
    
    //Private functions
    void goForward();
    void goBackward();
    void turnLeft();
    void turnRight();
    void stop();
    
  public :
    Robot();
    
    //To test rover step by step
    /*1*/ void Debug_Motor();
    /*2*/ void Debug_Mobility();
    /*3*/ void Debug_Encoder();
    /*4*/ void Debug_UltrasonicSensors();
    /*5*/ void Debug_Gyroscope();
    /*6*/ void Debug_AllSensor();
    /*7*/ void Debug_AutomaticMode();
    
    //Then you can use the rover procedures below. Let's play ;)
    void manualProcedure();
    void automaticProcedure();
};
#endif
