#include "CtrlLoop.h"
#include "Arduino.h"

CtrlLoop::CtrlLoop(char _id, Encoder *mEncoder, Motor *_motor, PID *_pid, double &_Setpoint, double &_Input, double &_Output)
: id(_id), encoder(mEncoder), motor(_motor), pid(_pid), Setpoint(_Setpoint), Input(_Input), Output(_Output)
{
	pid->SetMode(AUTOMATIC);
	this->Setpoint = 0;
}

CtrlLoop::CtrlLoop(char id, Encoder *mEncoder, Motor *_motor, double K_p, double K_i, double K_d)
: id(id), encoder(mEncoder), motor(_motor)
{
	
	//TODO: Fix this section to initialize member object pid
	pid = new PID (&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
	//_pid.SetMode(AUTOMATIC);
	//pid = &_pid;
	
	pid->SetMode(AUTOMATIC);
	//motor = _motor;
	
	//TODO: Fix this by initializing Setpoint first
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
//		Serial.print("Reverse controller mode");

		//TODO: replace with goBackward function
		motor->goBackward();

		//xMotor.goForward();

//		Serial.print("Go backward");
	}
	else
	{
		pid->SetControllerDirection(DIRECT);
//		Serial.print("DIRECT Controller Mode");
		//TODO: replace with goBackward function
		motor->goForward();
//		Serial.print("Go forward");
	}
}
