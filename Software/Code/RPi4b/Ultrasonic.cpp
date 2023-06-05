#include <iostream>
#include <wiringPi.h>
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int triggerPin, int echoPin, long unsigned int timeoutToSet){
    trigger=triggerPin;
    echo=echoPin;
    timeout=timeoutToSet;
    pinMode(trigger,OUTPUT);
    pinMode(echo,INPUT);
    digitalWrite(trigger,LOW);
}

void Ultrasonic::read(){
    delay(10);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    now=micros();
    while (digitalRead(echo) == LOW && micros()-now<timeout);
    recordPulseLength();
    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;
}

double Ultrasonic::getDistance(){ return distanceMeters; }

void Ultrasonic::recordPulseLength(){
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH );
    endTimeUsec = micros();
}
