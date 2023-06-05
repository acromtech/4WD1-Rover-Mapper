#ifndef Encoder_h
#define Encoder_h

#include <iostream>
#include <wiringPi.h>
#include <unistd.h>

/**
 * To active on-board RaspberryPi ISR  follow the steps below :
 * 1. Open a terminal and put the following commands :
 *    sudo apt-get update
 *    sudo apt-get upgrade wiringpi
 * 2. Check free GPIO pin thanks to the folowing command and save the wiringPi number
 *    gpio readall
 * 3. Open the /boot/config.txt file thanks to the following command :
 *    sudo nano /boot/config.txt
 * 4. Put the following lines into the file and replace <GPIO_NUMBER> by your saved GPIO number
 *    dtoverlay=gpio-irq,<GPIO_NUMBER>=active
 * 5. Save modifications and reboot system :
 *    sudo reboot
*/

class Encoder {
private:
  //Private attributes
  int pos;
  int pinA, pinB;

public:
  //Public functions
  Encoder(int pinAIn,int pinBIn);
};

#endif
