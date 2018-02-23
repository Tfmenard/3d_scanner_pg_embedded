/*
  Motor.h - Library for flashing Motor code. 
  */
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
    char motorID, motorDirection, motorPosition, motorSpeed;//class variables
    double speedPercentage;
    int pwm;//speed
    
    //class functions
    Motor(char command, char id, char mDirection, char mPosition, char mSpeed);
    void goForward();
    void goBackward();
    void go();//this function works according to the set direction
    void setMotorSpeed(char inputMotorSpeed);
    void setMotorDirection(char inputMotorDirection);
    void stopMotor(); 

    //functions not used:
    //void goToPosition(double desiredPosition);
    //double readPosition();
    //void setPins();//set pins depending on motor id

    //variables not used:
    //command
  private:
    int pwm_pin, InA, InB;
};

#endif
