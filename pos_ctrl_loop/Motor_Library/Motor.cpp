/*
  Motor.cpp - Library for flashing Motor code.
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(char command, char id, char mDirection, char mPosition, char mSpeed)
{
  motorID = id;
  motorDirection = mDirection;
  motorPosition = mPosition;
  motorSpeed = mSpeed;
  speedPercentage = (motorSpeed - '0')/10;// if 0, speed is 0, else pwm = 255*speedPercentage.

   //set correct speed with pwm: (25% = 64; 50% = 127; 75% = 191; 100% = 255)
  if (motorSpeed = '0')
  {
     pwm = 0; 
  } else 
     pwm = 255*speedPercentage; //transforming double to int
    
  //set correct pins depending on ID: pwm, inA, inB. 
  if (motorID = 'l') 
  {
    InA = 26;
    InB = 27;
    pwm_pin = 2;
  }
  else if (motorID = '2')
  {
    InA = 36;
    InB = 37;
    pwm_pin = 3;
  }

}

void Motor::goForward()
{
 analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, HIGH);
    digitalWrite(InB, LOW);
    motorDirection = '1';
}

void Motor::goBackward()
{
  analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, HIGH);
    motorDirection = '2';
}

void Motor::go()
{
  if (motorDirection = '1')//go forward
  {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, HIGH);
    digitalWrite(InB, LOW);
   } 
   else if (motorDirection = '2')//go backward
   {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, HIGH);
   }
   else if (motorDirection = '0')
   {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, LOW);
   }  
     
}

void Motor::setMotorSpeed(char inputSpeedPercentage)
{
  motorSpeed = inputSpeedPercentage;
  speedPercentage = (motorSpeed - '0')/10;// if 0, speed is 0, else pwm = 255*speedPercentage.;
  pwm = speedPercentage*255; 
}
void Motor::setMotorDirection(char inputMotorDirection)
{
  motorDirection = inputMotorDirection;
}
