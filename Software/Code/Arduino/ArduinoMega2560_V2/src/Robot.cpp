#include <Arduino.h>
#include <Ultrasonic.h>
#include <Gyroscope.h>
#include <Joystick.h>
#include <Motor.h>
#include <Encoder.h>
#include <SimplePID.h>
#include <Robot.h>

Robot::Robot() :  usF(FRONT_ULTRASONIC_TRIGGER_PIN,FRONT_ULTRASONIC_ECHO_PIN,10000), 
                  usB(BACK_ULTRASONIC_TRIGGER_PIN,BACK_ULTRASONIC_ECHO_PIN,10000), 
                  usR(RIGHT_ULTRASONIC_TRIGGER_PIN,RIGHT_ULTRASONIC_ECHO_PIN,10000) 
                  {}

void Robot::init(){
  Serial.begin(DEBUG_BAURATE);                              //Serial PIN : 0(RX), 1(TX) - https://www.arduino.cc/reference/en/language/functions/communication/serial/
  Serial3.begin(BLUETOOTH_BAURATE);                         //Serial3 PIN : 15(RX) 14(TX) - https://www.arduino.cc/reference/en/language/functions/communication/serial/ - for Bleutooth connection

  joystick.init(0,0);                                       //joyOffsetIn,switIn
  gyro.init();                                              //A4(SDA) A5(SCL) - https://www.arduino.cc/reference/en/language/functions/communication/wire/
  gyro.calibration();
  motorL.init(LEFT_MOTOR_PIN);
  motorR.init(RIGHT_MOTOR_PIN);
  encoderL.init(LEFT_ENCODER_PINA,LEFT_ENCODER_PINB);       //Green(interrupt pin),Yellow
  encoderR.init(RIGHT_ENCODER_PINA,RIGHT_ENCODER_PINB);     //Green(interrupt pin),Yellow
  pidL.setParams(PID_PARAMETER_KP,PID_PARAMETER_KD,PID_PARAMETER_KI,PID_PARAMETER_UMAX);
  pidR.setParams(PID_PARAMETER_KP,PID_PARAMETER_KD,PID_PARAMETER_KI,PID_PARAMETER_UMAX);
}

void Robot::manualProcedure(){
  /****************************/
  /* Read inputs              */
  /****************************/
  joystick.read();

  usF.read();
  usB.read();
  usR.read();

  if (usF.getDistance()<=SECU_DISTANCE)    Of=1; else Of=0;
  if (usB.getDistance()<=SECU_DISTANCE)    Ob=1; else Ob=0;
  if (usR.getDistance()<=SECU_DISTANCE)    Or=1; else Or=0;

  /****************************/
  /* Write outputs            */
  /****************************/
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
}

void Robot::automaticProcedure(){
    // repositionnement du robot à DISTANCE_SECU du mur ( correction de trajectoire asservicement en position réaliser sur les roues puis sur le la position des capteurs à l'obstacle)
    
    usF.read(CM);
    usB.read(CM);
    usR.read(CM);
    
    // lecture des entrées : Ob, Oh, Og, Od, init? (lecture des capteurs : obstacle si capteurs à pos <= DISTANCE_SECU)
    if (usF.getDistance()<=FRONT_SECU_DISTANCE)   Of=1; else Of=0;
    if (usB.getDistance()<=SECU_DISTANCE)         Ob=1; else Ob=0;
    if (usR.getDistance()<=SECU_DISTANCE)         Or=1; else Or=0;

    gyro.read();

    /************/
    /* Bloc F  */
    /************/
    switch(CurrentState){
        case 0 :  if(Or && Ob)                          NextState = 1;   break;
        case 1 :  if(!Of)                               NextState = 2;   break;
        case 2 :  if(Of){ //TURN LEFT
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    NextState = 3;
                    robotOrientation=(gyro.getZDeg())+90-SPEED_MARGIN;
                  }
                  else if (!Or){ //TURN RIGHT
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    NextState = 4;
                    robotOrientation=(gyro.getZDeg())-90+SPEED_MARGIN;
                  }
                  else if (usR.getDistance()<SECU_DISTANCE-WALL_DISTANCE_MARGIN){ //LEFT COMPENSATION
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    robotOrientation=(gyro.getZDeg())+COMPENSATION_ANGLE;
                    NextState = 5;
                  }
                  else if (usR.getDistance()>SECU_DISTANCE+WALL_DISTANCE_MARGIN){ //RIGHT COMPENSATION
                    motorL.setMotor(STOP,0);
                    motorR.setMotor(STOP,0);
                    robotOrientation=(gyro.getZDeg())-COMPENSATION_ANGLE;
                    NextState = 6; 
                  }
                  break;
        case 3 :  if(robotOrientation<=gyro.getZDeg())   NextState = 11;  break;  //LEFT
        case 4 :  if(robotOrientation>=gyro.getZDeg())   NextState = 11;  break;  //RIGHT
        case 5 :  if(robotOrientation<=gyro.getZDeg())   NextState = 2;   break;  //LEFT
        case 6 :  if(robotOrientation>=gyro.getZDeg())   NextState = 2;   break;  //RIGHT
        case 11 : if(Or)                                 NextState = 2;   break;
        default :                                                         break;
        
    }
    
    /************/
    /* Bloc M  */
    /************/   
    if(initState){
        CurrentState = 0;
        initState = 0;
        gyro.calibration();
    }
    else CurrentState = NextState;

    //AutomaticModeDebugSerial();
    
    /************/
    /* Bloc G   */
    /************/
    B = 0;
    F = (CurrentState == 2) || (CurrentState == 11);
    L = (CurrentState == 3);
    R = (CurrentState == 4);
    S = 0;
    SL = (CurrentState == 5);
    SR = (CurrentState == 6);
    
    /****************************/
    /* Ecriture des sorties     */
    /****************************/
    //Association des actions aux commandes moteurs
    if(B){
      motorL.setMotor(BACKWARD,MAX_SPEED);
      motorR.setMotor(FORWARD,MAX_SPEED);
    }
    else if(F){
      motorL.setMotor(FORWARD,MAX_SPEED);
      motorR.setMotor(BACKWARD,MAX_SPEED);
    }
    else if(R){
      motorL.setMotor(FORWARD,MAX_SPEED);
      motorR.setMotor(FORWARD,MAX_SPEED);
    } 
    else if(L){
      motorL.setMotor(BACKWARD,MAX_SPEED);
      motorR.setMotor(BACKWARD,MAX_SPEED);
    }
    else if(S){
      motorL.setMotor(STOP,0); 
      motorR.setMotor(STOP,0);
    }
    else if(SL){
      motorL.setMotor(STOP,0);
      motorR.setMotor(BACKWARD,MAX_SPEED);
    }
    else if(SR){
      motorL.setMotor(FORWARD,MAX_SPEED);
      motorR.setMotor(STOP,0);
    }
}

void Robot::Debug_AllSensor(){
    joystick.read();
    gyro.read();
    usF.read(CM);
    usB.read(CM);
    usR.read(CM);
    Serial.print(encoderL.getEncoderPos());
    Serial.print(" , ");
    Serial.print(encoderR.getEncoderPos());
    Serial.print(" , ");
    Serial.print(joystick.getAngleToSet());
    Serial.print(" , ");
    Serial.print(joystick.getSpeedToSet());
    Serial.print(" , ");
    Serial.print(gyro.getZDeg());
    Serial.print(" , ");
    Serial.print(usF.getDistance());
    Serial.print(" , ");
    Serial.print(usB.getDistance());
    Serial.print(" , ");
    Serial.println(usR.getDistance());
}

void Robot::Debug_AutomaticMode(){
    Serial.print(CurrentState);
    Serial.print(" , ");
    Debug_AllSensor();
    Serial.print(" , ");
    Serial.print(Of);
    Serial.print(" , ");
    Serial.print(Ob);
    Serial.print(" , ");
    Serial.print(Or);
}