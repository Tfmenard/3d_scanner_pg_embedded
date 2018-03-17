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


void CtrlLoop::homing()
{
	this->pid->SetMode(MANUAL);

	this->motor->pwm = 100;
	this->motor->goBackward();
	
	while (digitalRead(30) == 0)
	{
	}

	encoder->write(0);
	Setpoint = 0;
	
	this->motor->stopMotor();
	this ->pid->SetMode(AUTOMATIC);
}

void CtrlLoop::sendFBackStreamIfMoving()//Streams encoder position only when the motor is moving
{
  bool close_enough = (Input > (Setpoint - posThreshold)) && (Input  < (Setpoint + posThreshold));
  if(motor->isMoving && close_enough)
  {
    //Print status
     // Serial.print("ECD,");
	 // Serial.print(motor->motorID);
	 // Serial.print(',');
      //Serial.print(Input/motor->gear_ratio);
      //Serial.print(',');
      //Serial.print('\n'); 
	String cmd = "DMC,";
    cmd += motor->motorID;
    cmd += ',';
    cmd += Input/motor->gear_ratio;
    cmd += ',';
    cmd += '\n';
	Serial.print(cmd);
    motor->isMoving = false; 
  }
  
  if (close_enough)
  {
    
    //motor->isMoving = false;
    //Reset values to only print values once
    //device = "";
    //string_id = "";
    //value = "";
  }
  else if(motor->isMoving && !close_enough)
  {
      //Print status
		String cmd = "ECD,";
		cmd += motor->motorID;
		cmd += ',';
		cmd += Input/motor->gear_ratio;
		cmd += ',';
		cmd += '\n';
		streamCounter = (streamCounter++)%10;
		if(streamCounter == 0)
		{
			Serial.print(cmd);
		}	
  }
}