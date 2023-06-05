#ifndef Motor_h
#define Motor_h

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4

#define PWR_MAX_FOREWARD 600
#define PWR_MAX_BACKWARD 250
#define PWR_STOP 400

class Motor{
  private :
    int pin;
  public :
    Motor(unsigned int PWMpin);
    void setMotor(int dir,int pwr);
    void testMotor();
    int getMotorPin();
};
#endif 
