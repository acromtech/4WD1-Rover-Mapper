#include "Ultrasonic.h"

Ultrasonic usF(30, 31); // Trig & Echo
Ultrasonic usB(32, 33);
Ultrasonic usR(26, 27);

void setup() {
  Serial.begin(9600);
}

void loop () {
  Serial.print(usF.read(CM));
  Serial.print(" cm , ");
  Serial.print(usB.read(CM));
  Serial.print(" cm , ");
  Serial.print(usR.read(CM));
  Serial.println(" cm , ");
  delay(300);
}
