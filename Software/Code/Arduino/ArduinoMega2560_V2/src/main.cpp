#include <Arduino.h>
#include "Robot.h"

Robot robot;

void setup() {
  robot.init();
}

void loop() {
  /*
  if(MANUAL_MODE) robot.manualProcedure();
  else robot.automaticProcedure();
  */
  robot.Debug_AllSensor();
} 