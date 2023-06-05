#ifndef Joystick_h
#define Joystick_h

#define MIN_VAL 0
#define MAX_VAL 255

class Joystick{
  private :
    int val;
    int swit;
    int joyOffset;
    double angle,resultatAngle;
    float old_x_joystick, old_y_joystick;
    float V, new_x_joystick, new_y_joystick;
  public :
    Joystick();
    void init(int joyOffsetIn, int switIn);
    void read();
    float joyRawToPhysY(int raw);
    float joyRawToPhysX(int raw);
    float vitesse(int x, int y);
    double AlKashi(int x, int y);
    void handle_joystick(bool new_val);
    double getAngleToSet();
    float getSpeedToSet();
};
#endif 
