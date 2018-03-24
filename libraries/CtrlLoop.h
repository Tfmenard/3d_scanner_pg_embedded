/*
CtrlLoop.h - Library to handle Motor, Encoder with PID Control computations.
input: reading from encoder
setpoint (desired position)
output: computed by pid
*/
#ifndef CtrlLoop_h
#define CtrlLoop_h

#include "Motor.h"
#include "Encoder.h"
#include <PID_v1.h>


class CtrlLoop
{
public:
	double Setpoint = 0, Input = 0, Output = 0;
	double Kp = 5, Ki = 0, Kd = 0;
	long oldPosition = -999;
	long newPosition = -999;
	char id;
	bool isHoming = false;	
	double posThreshold = 1;
	int streamCounter = 0;

	Encoder *encoder;
	PID *pid;
	Motor *motor;

	//class functions
	CtrlLoop(char id, Encoder *mEncoder, Motor *_motor, double K_p, double K_i, double K_d);
	CtrlLoop(char _id, Encoder *mEncoder, Motor *_motor, PID *_pid, double &_Setpoint, double &_Input, double &_Output);
	void updatePosition();
	void checkIfHomingDone(int switchPin);
	void findMotorDirection();
	bool updatePID();

	void homing();

	void go();//this function works according to the set direction
	void setMotorSpeed(char inputMotorSpeed);
	void setMotorDirection(char inputMotorDirection);
	void setupPins();
	void stopMotor();
	void sendFBackStreamIfMoving();
	void sendFeedBackStatus(String cmd_id);

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
