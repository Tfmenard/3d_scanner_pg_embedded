/*
   This file handles Motor 3 and Motor 4: X and Servo Motors
   Author: Gabriel Chootong
   Date: March 6, 2018
   Motor X: pwm pin(2), InA(26), InB(27)
   Encoder X: 18,19
   Servo: 11
   
 */



// Change this path to include libraries
#define PROJECT_ROOT C:\Users\royni\Desktop\3d_scanner_pg_embedded-message-cmds\libraries

#define TO_STRING(s) #s
#define ABSOLUTE_PATH(root, relative_path) TO_STRING(root\relative_path)
#define RELATIVE_PATH(library) ABSOLUTE_PATH(PROJECT_ROOT, library)

#include RELATIVE_PATH(CtrlLoop.h)
#include RELATIVE_PATH(CtrlLoop.cpp)
#include RELATIVE_PATH(Motor.h)
#include RELATIVE_PATH(Motor.cpp)
 
#include <Servo.h>

int PWM_pin_X = 2;
Encoder X_encoder(18, 19);
double Kp = 5, Ki = 0, Kd = 0;
Servo servo_motor;

bool pidComputedX = false;
Motor X_motor('X', '1', 'd');
CtrlLoop X_ctrlLoop('B', &X_encoder, &X_motor, Kp, Ki, Kd);

String PC_input, device, string_id, value;
double desiredPosition;
double positionThreshold = 0.1;
char id;
long encoder_position;

void setup() {
  Serial.begin(115200);

  X_ctrlLoop.updatePosition();

  servo_motor.attach(11);//attaches servo on pin 11 to the servo object.
  
  Serial.println("Ready");

}

void loop() {
  
  X_ctrlLoop.updatePosition();
  X_ctrlLoop.findMotorDirection();

  while (Serial.available() > 0) 
  { 
    PC_input = Serial.readStringUntil('\n');//read input from pc
    execute_command(PC_input);//here parse pc command and execute it
    
  }

  pidComputedX = X_ctrlLoop.pid->Compute();//must run once in every void loop iteration
  analogWrite(PWM_pin_X, X_ctrlLoop.Output);

  //send motor done signal if motor is close enough
  if (device == "M")
  {
    if (string_id == "X")
    {
      printIfCloseEnough(X_ctrlLoop);
    }
  }
  
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
      if (string_id == "X")// X motor selected
      {
        X_ctrlLoop.Setpoint = desiredPosition*X_motor.gear_ratio;//for X motor (X motor control loop)
        Serial.print("Updated setpoint on X motor: ");
        Serial.print(X_ctrlLoop.Setpoint);
        Serial.print('\n');
        Serial.print("Done");
        Serial.print('\n');
                
      } 
      
    } else if (device == "E") //encoder command
    {
      if (string_id == "X")
      {
        Serial.print("X encoder position value is ");
        Serial.print(X_ctrlLoop.Input);
        Serial.print('\n');
        Serial.print("Done");
        Serial.print('\n');

      } else if (string_id == "S")
      {
        Serial.print("Last position command for servo motor is (degrees): ");
        Serial.print(servo_motor.read());
        Serial.print('\n');
        Serial.print("Done");
        Serial.print('\n');
      }
    } else if (device == "S") //servo command
    {
      servo_motor.write(desiredPosition);
      delay(1000);//camera delay
      
      Serial.print("Updated servo motor position (degrees): ");
      Serial.print(servo_motor.read());
      Serial.print('\n');
      Serial.print("Done");
      Serial.print('\n');
    } else //invalid command
      { 
        Serial.print("Invalid command. Format must be: Device,ID,Position ");
        Serial.print('\n');
      }
  

}

void printIfCloseEnough(CtrlLoop control_loop)//prints back done signal when motor is close enough to desired position
{
  bool close_enough = (control_loop.Input > (control_loop.Setpoint - positionThreshold)) && (control_loop.Input  < (control_loop.Setpoint + positionThreshold));
  if (close_enough)
  {
    Serial.print("Motor ");
    Serial.print(string_id);
    Serial.print(" is within threshold");
    Serial.print('\n');
    Serial.print("Done");
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