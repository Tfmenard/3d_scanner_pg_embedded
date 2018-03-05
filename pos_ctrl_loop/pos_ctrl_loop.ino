/*
 * This file handles Motor 1 and Motor 2 (both DC motors)
Things to do:
- add functionality for a second motor (initialize pins, etc)
- add pc input parsing
- add second encoder?
- check all parameters are correct type (on this .ino and on the library) i.e. string-string, char-char...
- call motors X, Y, S, B. encoders are called E. check this
*/
#include "Motor_Library/CtrlLoop.h"

int PWM_pin_base = 2, InA1 = 26, InB1 = 27;
int PWM_pin_Y = 3;
int PWM_val = 0;
Encoder base_encoder(18, 19);
Encoder Y_encoder(20, 21);
//double Setpoint, Input, Output;
double Kp = 5, Ki = 0, Kd = 0;
bool pidComputedBase = false;
bool pidComputedY = false;

Motor base_motor('B','1','d');
Motor Y_motor('Y', '1', 'd');
CtrlLoop base_ctrlLoop('B', &base_encoder, &base_motor, Kp, Ki, Kd);
CtrlLoop Y_ctrlLoop('Y', &Y_encoder, &Y_motor, Kp, Ki, Kd);

String PC_input, device, string_id, value;
double desiredPosition;
char id;
long encoder_position;

void setup() {
  Serial.begin(115200);

  base_ctrlLoop.updatePosition();
  Y_ctrlLoop.updatePosition();
  
  Serial.println("Ready");

}

void loop() {
  device = "";
  
  base_ctrlLoop.updatePosition();
  Y_ctrlLoop.updatePosition();
  
//  Serial.print("Input Address Inside Loop ");
//  Serial.print((long) &base_ctrlLoop.Input, HEX);
//  Serial.print(", Input Value Inside Loop ");
//  Serial.println(base_ctrlLoop.Input);
//
//  Serial.print('\n');
//  Serial.print("Input Address Inside Loop ");
//  Serial.print((long) &Y_ctrlLoop.Input, HEX);
//  Serial.print(", Input Value Inside Loop ");
//  Serial.println(Y_ctrlLoop.Input);

  
  base_ctrlLoop.findMotorDirection();
  Y_ctrlLoop.findMotorDirection();
  while (Serial.available() > 0) { //here read and parse pc input. 
   PC_input = Serial.readStringUntil('\n');
    
    //splitting PC_input:
    device = getValue(PC_input, ',', 0);
    string_id = getValue(PC_input, ',', 1);
    value = getValue(PC_input, ',', 2);

    //converting strings to appropriate type for controlLoop class
    desiredPosition = value.toDouble();
    Serial.print(PC_input);
    Serial.print('\n');
    Serial.print(device);
    Serial.print('\n');
     Serial.print(string_id);
    Serial.print('\n');
     Serial.print(value);
    Serial.print('\n');
    
    if (device == "M") //motor command
   {
    if (string_id == "B")// Base motor selected
     {
      base_ctrlLoop.Setpoint = desiredPosition;//for motor 1 (or controlLoop 1) 
         Serial.print("Updated setpoint on base motor: ");
         Serial.print(base_ctrlLoop.Setpoint);
         Serial.print('\n');
         Serial.print("Done");
         Serial.print('\n');
         
     } else if (string_id == "Y")//Y motor selected
       {
         Y_ctrlLoop.Setpoint = desiredPosition;//for motor 2 (or controlLoop 2)
         Serial.print("Updated setpoint on Y motor: ");
         Serial.print(Y_ctrlLoop.Setpoint);
         Serial.print('\n');
         Serial.print("Done");
         Serial.print('\n');


       }       
    } else if (device == "E") //encoder command
    { 
      if (string_id == "B")
      {
        Serial.print("Base encoder position value is ");
        Serial.print(base_ctrlLoop.Input);
        Serial.print('\n');
        Serial.print("Done");
        Serial.print('\n');

       } else if (string_id == "Y")
       {
          Serial.print("Y encoder position value is ");
          Serial.print(Y_ctrlLoop.Input);
          Serial.print('\n');
          Serial.print("Done");
          Serial.print('\n');
        }
      }
  
  }
//   if (device = "M") //motor command
//   {
//    if (id = 'B')// Base motor selected
//     {
//      base_ctrlLoop.Setpoint = desiredPosition;//for motor 1 (or controlLoop 1) 
//         Serial.print("Updated setpoint on base motor: ");
//         Serial.print(base_ctrlLoop.Setpoint);
//         Serial.print('\n');
//         Serial.print("Done");
//         Serial.print('\n');
//     } else if (id = 'Y')//Y motor selected
//       {
//         Y_ctrlLoop.Setpoint = desiredPosition;//for motor 2 (or controlLoop 2)
//         Serial.print("Updated setpoint on Y motor: ");
//         Serial.print(Y_ctrlLoop.Setpoint);
//         Serial.print('\n');
//         Serial.print("Done");
//         Serial.print('\n');
//
//
//       }       
//    } else if (device = 'E') //encoder command
//    { 
//      if (id = 'B')
//      {
//        Serial.print("Base encoder position value is ");
//        Serial.print(base_ctrlLoop.Input);
//        Serial.print('\n');
//        Serial.print("Done");
//        Serial.print('\n');
//
//       } else if (id = 'Y')
//       {
//          Serial.print("Y encoder position value is ");
//          Serial.print(Y_ctrlLoop.Input);
//          Serial.print('\n');
//          Serial.print("Done");
//          Serial.print('\n');
//        }
//      }
     
//   else if (device = '1')//servo command
//      {
//        //ctrlLoopServo.Setpoint = desiredPosition;
//      } else if (device = '2')
//      {
//        
//        }
      


    pidComputedBase = base_ctrlLoop.pid->Compute();//must run once in every void loop iteration
    pidComputedY = Y_ctrlLoop.pid->Compute();//must run once in every void loop iteration

    //get position feedback here
//    encoder_position = myEnc.read();
//    Serial.print(encoder_position + '\n');

    //TODO: Remove this line since handled in MotorClass
    analogWrite(PWM_pin_Y, Y_ctrlLoop.Output);
    analogWrite(PWM_pin_base, base_ctrlLoop.Output);

    //TODO: Debugging section should be made conditional
//    Serial.print("SP: ");
//    Serial.print(xCtrlLoop.Setpoint);
//    Serial.print(", In: ");
//    Serial.print(xCtrlLoop.Input);
//    Serial.print(", Out: ");
//    Serial.println(xCtrlLoop.Output);
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
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
