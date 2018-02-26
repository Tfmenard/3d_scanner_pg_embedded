#include "CtrlLoop.h"
#include "Arduino.h"


CtrlLoop::CtrlLoop(char id, Encoder *mEncoder, Motor *_motor, double K_p, double K_i, double K_d)
{
	encoder = mEncoder;
	PID _pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
	_pid.SetMode(AUTOMATIC);
	pid = &_pid;
	motor = _motor;
	this->Setpoint = 0;
}

void CtrlLoop::updatePosition()
{
	newPosition = encoder->read();
	if (newPosition != oldPosition) {
		oldPosition = newPosition;
		Input = newPosition;
	}
}

bool CtrlLoop::updatePID()
{
	this->pid->Compute();
}

void CtrlLoop::findMotorDirection()
{
	if (Setpoint < Input)
	{
		pid->SetControllerDirection(REVERSE);
		Serial.print("Reverse controller mode");

		//TODO: replace with goBackward function
		motor->goBackward();

		//xMotor.goForward();

		Serial.print("GO backward");
	}
	else
	{
		pid->SetControllerDirection(DIRECT);
		Serial.print("DIRECT Controller Mode");
		//TODO: replace with goBackward function
		motor->goForward();
		Serial.print("Go forward");
	}
}