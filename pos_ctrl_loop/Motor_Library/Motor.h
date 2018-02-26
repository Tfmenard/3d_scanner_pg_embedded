/*
  Motor.h - Library for flashing Motor code. 
  */
#ifndef Motor_h
#define Motor_h


#include "Arduino.h"

class Motor
{
  public:
    char motorID, motorDirection, motorPosition;//member variables
    double speedPercentage;
    int pwm;//speed
    
    //class functions
    Motor(char id, char mDirection, char mPosition);
    void goForward();
    void goBackward();
    void go();//this function works according to the set direction
    void setMotorDirection(char inputMotorDirection);
	void setupPins();
    void stopMotor(); 

    //functions not used:
    //void goToPosition(double desiredPosition);
    //double readPosition();
    //void setPins();//set pins depending on motor id

    //variables not used:
    //command
  public:
    int pwm_pin, InA, InB;
};

#endif
