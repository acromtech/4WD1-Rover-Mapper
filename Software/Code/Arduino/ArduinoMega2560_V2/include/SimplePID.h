#ifndef SimplePID_h
#define SimplePID_h

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4

class  SimplePID{
  private :
    float kp,kd,ki,umax;    //Parameters
    float eprev, eintegral; //Storage
  public :
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);
  void evalu(int value, int target, float deltaT, int &pwr, int &dir);
  void PIDAllWheeltest();
};
#endif 
