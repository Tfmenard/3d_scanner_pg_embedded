/*
   This file handles Motor 1 and Motor 2 (both DC motors): Base and Y Motors
   Author: Gabriel Chootong
   Date: March 6, 2018
   Base Motor: pwm pin(2), InA(26), InB(27)
   Y Motor: pwm pin(3), InA(36), InB(37)
   Encoder Base: pin (18,19)
   Encoder Y: pin (20,21)

*/


// Change this path to include libraries
#define PROJECT_ROOT C:\Users\Gabriel\Desktop\Deberes McGill\DP2\Github\3d_scanner_pg_embedded\libraries
#define TO_STRING(s) #s
#define ABSOLUTE_PATH(root, relative_path) TO_STRING(root\relative_path)
#define RELATIVE_PATH(library) ABSOLUTE_PATH(PROJECT_ROOT, library)

#include RELATIVE_PATH(CtrlLoop.h)
#include RELATIVE_PATH(CtrlLoop.cpp)
#include RELATIVE_PATH(Motor.h)
#include RELATIVE_PATH(Motor.cpp)

int PWM_pin_base = 2;
int PWM_pin_Y = 3;
Encoder base_encoder(18, 19);
Encoder Y_encoder(20, 21);
double Kp = 5, Ki = 0, Kd = 0;

bool pidComputedBase = false;
bool pidComputedY = false;
Motor base_motor('B', '1', 'd');
Motor Y_motor('Y', '1', 'd');
CtrlLoop base_ctrlLoop('B', &base_encoder, &base_motor, Kp, Ki, Kd);
CtrlLoop Y_ctrlLoop('Y', &Y_encoder, &Y_motor, Kp, Ki, Kd);

String PC_input, device, string_id, value;
double desiredPosition;
double positionThreshold = 0.6;
char id;
long encoder_position;
int switchPin = 30;

void setup() {
  Serial.begin(115200);

  base_ctrlLoop.updatePosition();
  Y_ctrlLoop.updatePosition();
  pinMode(switchPin, INPUT_PULLUP);

//  Serial.println("Ready");

}

void loop() {

  base_ctrlLoop.updatePosition();
  Y_ctrlLoop.updatePosition();
  base_ctrlLoop.findMotorDirection();
  Y_ctrlLoop.findMotorDirection();

  while (Serial.available() > 0)
  {
    PC_input = Serial.readStringUntil('\n');
    execute_command(PC_input);//here parse pc command and execute it

  }
  
  Y_ctrlLoop.checkIfHomingDone(switchPin);
  
  pidComputedBase = base_ctrlLoop.pid->Compute();//must run once in every void loop iteration
  pidComputedY = Y_ctrlLoop.pid->Compute();//must run once in every void loop iteration

  analogWrite(PWM_pin_Y, Y_ctrlLoop.Output);
  analogWrite(PWM_pin_base, base_ctrlLoop.Output);

}

void execute_command(String command)
{
  //splitting PC_input:
  device = getValue(command, ',', 0);
  string_id = getValue(command, ',', 1);
  value = getValue(command, ',', 2);

  //converting strings to appropriate type for controlLoop class
  desiredPosition = value.toDouble();

  if (device == "M") //motor command
  {
    if (string_id == "B")// Base motor selected
    {
      if(value == "R")
      {

      }
      else
      {
        base_ctrlLoop.Setpoint = desiredPosition*base_motor.gear_ratio;//for base motor (base control loop) 

        //Command received status
        //TODO: Implement protocol on receiving end before using this status.
        //        Serial.print("MCD,B,");
        //        Serial.print(base_ctrlLoop.Setpoint);
        //        Serial.print('\n');
      }
    } 
    else if (string_id == "Y")//Y motor selected
     { 
      if (value == "H") 
        {
          Y_ctrlLoop.homing();
          base_ctrlLoop.homing(); 
        }
      else if(value == "R")
      {

      }
      else
      {
        double Y_gear_ratio = Y_motor.gear_ratio;
        Y_ctrlLoop.Setpoint = desiredPosition * Y_gear_ratio;//for Y motor (Y control loop)
      
      }
     } 
  } 
  else if (device == "E") //encoder command
  {
    if (string_id == "B")
    {
      Serial.print("ECD,B,");
      Serial.print(base_ctrlLoop.Input);
      Serial.print('\n');

    } else if (string_id == "Y")
    {
      Serial.print("ECD,Y,");
      Serial.print(Y_ctrlLoop.Input);
      Serial.print('\n');
    }
  } else //invalid command
  {
//    Serial.print("Invalid command. Format must be: Device,ID,Position ");
//    Serial.print('\n');
  }

  device = "";
  string_id = "";
  value = "";
  
}

void printIfCloseEnough(CtrlLoop control_loop)//prints back done signal when motor is close enough to desired position
{
  bool close_enough = (control_loop.Input > (control_loop.Setpoint - positionThreshold)) && (control_loop.Input  < (control_loop.Setpoint + positionThreshold));
  if (close_enough)
  {
    //DM,B,encoder_position and end with \n
    Serial.print("DM,");//done motor signal
    Serial.print(string_id);
    Serial.print(",");
    Serial.print(control_loop.Input);
    Serial.print('\n');

    //Reset values to only print values once
    device = "";
    string_id = "";
    value = "";
  }
}

String getValue(String data, char separator, int index)//this method is used to split string into substrings
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
