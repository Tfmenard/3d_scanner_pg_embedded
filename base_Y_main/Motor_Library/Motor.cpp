/*
  Motor.cpp - Library for flashing Motor code.
*/

#include "Motor.h"

Motor::Motor(char id, char mDirection, char mPosition)
{
  motorID = id;
  motorDirection = mDirection;
  motorPosition = mPosition;

    
    
  //set correct pins depending on ID: pwm, inA, inB. 
  if (motorID == 'l') 
  {
    InA = 26;
    InB = 27;
    pwm_pin = 2;
  }
  else if (motorID == '2')
  {
    InA = 36;
    InB = 37;
    pwm_pin = 3;
  }
  else if(motorID == 'X')
  {
	InA = 26;
	InB = 27;
	pwm_pin = 2;
  }
  else if(motorID == 'B')
  {
  InA = 26;
  InB = 27;
  pwm_pin = 2;
  }
  else if(motorID == 'Y')
  {
  InA = 36;
  InB = 37;
  pwm_pin = 3;
  }

  setupPins();

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
  if (motorDirection == '1')//go forward
  {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, HIGH);
    digitalWrite(InB, LOW);
   } 
   else if (motorDirection == '2')//go backward
   {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, HIGH);
   }
   else if (motorDirection == '0')
   {
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, LOW);
   }  
     
}

void Motor::stopMotor()
{
    analogWrite(pwm_pin, abs(pwm));  
    digitalWrite(InA, LOW);
    digitalWrite(InB, LOW);
}

void Motor::setMotorDirection(char inputMotorDirection)
{
  motorDirection = inputMotorDirection;
}

void Motor::setupPins()
{
	pinMode(InA, OUTPUT);
	pinMode(InB, OUTPUT);
	pinMode(pwm_pin, OUTPUT);
}
