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
	if (id == 'Y')
{
isHoming = true;
Setpoint = (999999);//to ensure that Y motor rotates in the right direction
} 
else
{
	isHoming = true;
	Setpoint = (-999999);//arbitrary long distance
	
	if (id == 'B')
	{
		encoder->write(0);
          Setpoint = 0;
          isHoming = false;

	}
}
}

void CtrlLoop::checkIfHomingDone(int switchPin)
{
	if (digitalRead(switchPin) != 0 && (isHoming) )
  {
          encoder->write(0);
          Setpoint = 0;
          isHoming = false;
 
          //Serial.print("switch activated homing");
          //Serial.print('\n');
   }
}


void CtrlLoop::sendFBackStreamIfMoving()//Streams encoder position only when the motor is moving
{
  bool close_enough = (Input > (Setpoint - posThreshold)) && (Input  < (Setpoint + posThreshold));
  if(motor->isMoving && close_enough)
  {
    //Send current position status
	  sendFeedBackStatus("ECD,");
  }
  
  if (close_enough)
  {
    
    //do nothing
  }
  else if(motor->isMoving && !close_enough)
  {
	   //Send current position status
		sendFeedBackStatus("ECD,");
  }
}

void CtrlLoop::sendFeedBackStatus(String cmd_id)
{
	//Print current position status
	String cmdStringHead = cmd_id;
	cmdStringHead += motor->motorID;
	cmdStringHead += ',';
	double posAsDouble = Input/motor->gear_ratio;
	uint16_t posAsUint16 = static_cast<uint16_t>(posAsDouble + 0.5);
	String cmdStringTail = ",";
	char startMsgChar = '{';
	char endMsgChar = '}';
	
	double dividerAsDouble = posAsDouble/120.0;
	uint8_t dividerAsInt = (uint8_t)dividerAsDouble;
	int posAsInt = (int)posAsDouble;
	int remainderAsInt = posAsInt%120;
	uint8_t remainder = (uint8_t)remainderAsInt;
	uint8_t posHead = dividerAsInt;
	uint8_t posTail = remainder;
	
	Serial.write(startMsgChar);
	Serial.print(cmdStringHead);
	Serial.write(posHead);
	Serial.write(posTail);
	Serial.print(cmdStringTail);
	Serial.write(endMsgChar);
}