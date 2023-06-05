#include <wiringPi.h>
#include "Robot.h"

int main() {
  wiringPiSetup();
  Robot robot;
  while(1){
    robot.Debug_Gyroscope();
    //if(MANUAL_MODE) robot.manualProcedure();
    //else robot.automaticProcedure();
  }
  return 0;
}
