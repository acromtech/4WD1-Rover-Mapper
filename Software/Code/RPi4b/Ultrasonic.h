#ifndef Ultrasonic_h
#define Ultrasonic_h

class Ultrasonic{
  private:
    void recordPulseLength();
    int trigger;
    int echo;
    long unsigned int timeout;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;
  public:
    Ultrasonic(int triggerPin, int echoPin, long unsigned int timeoutToSet);
    void read();
    double getDistance();
};

#endif
