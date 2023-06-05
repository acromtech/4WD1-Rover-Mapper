#ifndef Joystick_h
#define Joystick_h

#define MIN_VAL 0
#define MAX_VAL 255

#define JOYOFFSET 0

class Joystick{
  private :
    //Private attributes
    int fd;
    int swit;
    uint8_t val;
    double angle,resultatAngle;
    float old_x_joystick, old_y_joystick;
    float V, new_x_joystick, new_y_joystick;
    
    //Private functions
    float customMap(float value, float inputMin, float inputMax, float outputMin, float outputMax);
    float joyRawToPhysY(int raw);
    float joyRawToPhysX(int raw);
    float vitesse(int x, int y);
    double AlKashi(int x, int y);
    void handle_joystick(bool new_val);
    
  public :
    //Public functions
    Joystick(const char* portName, int baudRate);
    ~Joystick();
    void read();
    double getAngleToSet();
    float getSpeedToSet();
};
#endif 
