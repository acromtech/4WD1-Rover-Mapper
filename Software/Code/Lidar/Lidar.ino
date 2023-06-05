/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3 
 */

/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include "RPLidar.h"


// You need to create an driver instance
RPLidar lidar;

#define RPLIDAR_MOTOR 7  // The PWM pin for control the speed of RPLIDAR's motor.
                         // This pin should connected with the RPLIDAR's MOTOCTRL signal

#define rxPin 10
#define txPin 11
#define rxPin1 12
#define txPin2 13

//SoftwareSerial bluetoothSerial(rxPin, txPin); // RX, TX pins

int cpt=0;
unsigned int val=0;
unsigned int angleInt=0;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600); // Initialise le port série logiciel pour la communication Bluetooth
  Serial1.begin(115200);
   lidar.begin(Serial1);
  Serial.print(" test ");
  // bind the RPLIDAR driver to the arduino hardware serial
 
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}
byte quinze = 15;
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
    float angle = lidar.getCurrentPoint().angle;        //anglue value in degree
    bool startBit = lidar.getCurrentPoint().startBit;   //whether this point is belong to a new scan
    byte quality = lidar.getCurrentPoint().quality;     //quality of the current measurement

    val= static_cast<unsigned int>(distance);
    angleInt=static_cast<unsigned int>(angle);

   // if ((angle >= 0 && angle <= 5) || (angle >= 355 && angle <= 360) || (angle >= 175 && angle <= 185)) { 

   if(quality>14){
    Serial3.write((byte*)&val, sizeof(val));
    Serial3.write((byte*)&angleInt, sizeof(angleInt)); 
    Serial.println(quality); 
    //Serial.println(val);
    //Serial.println(angleInt);
 
   }
  }else{

    analogWrite(RPLIDAR_MOTOR, 0);  //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();
      Serial.print(" erreur start ");
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}
