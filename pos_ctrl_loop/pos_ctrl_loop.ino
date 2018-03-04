#include "Motor_Library/CtrlLoop.h"

int PWM = 2, InA1 = 26, InB1 = 27;
int PWM_val = 0;
Encoder myEnc(18, 19);
//double Setpoint, Input, Output;
double Kp = 5, Ki = 0, Kd = 0;
bool pidComputed = false;

Motor xMotor('X','1','d');
CtrlLoop xCtrlLoop('X', &myEnc, &xMotor, Kp, Ki, Kd);

void setup() {
  Serial.begin(115200);

  xCtrlLoop.updatePosition();
  
  Serial.println("Ready");

}

void loop() {
  xCtrlLoop.updatePosition();

  Serial.print("Input Address Inside Loop ");
  Serial.print((long) &xCtrlLoop.Input, HEX);
  Serial.print(", Input Value Inside Loop ");
  Serial.println(xCtrlLoop.Input);


  
  xCtrlLoop.findMotorDirection();
  
  while (Serial.available() > 0) {
    xCtrlLoop.Setpoint = Serial.parseInt();
  }
  
    pidComputed = xCtrlLoop.pid->Compute();



    //TODO: Remove this line since handled in MotorClass
    analogWrite(PWM, xCtrlLoop.Output);


    //TODO: Debugging section should be made conditional
    Serial.print("SP: ");
    Serial.print(xCtrlLoop.Setpoint);
    Serial.print(", In: ");
    Serial.print(xCtrlLoop.Input);
    Serial.print(", Out: ");
    Serial.println(xCtrlLoop.Output);
}
