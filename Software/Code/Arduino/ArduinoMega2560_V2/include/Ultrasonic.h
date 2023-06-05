#ifndef Ultrasonic_h
#define Ultrasonic_h

#define CM 28
#define INC 71

class Ultrasonic {
  private:
    uint8_t trig;
    uint8_t echo;
    boolean threePins = false;
    unsigned long previousMicros;
    unsigned long timeout;
    unsigned int timing();
    float distance;
  public:
    Ultrasonic(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut); 
    void setTimeout(unsigned long timeOut);
    void read(uint8_t und = CM);
    void printDistanceSerial();
    float getDistance();
};
#endif
