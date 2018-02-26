//#include <Encoder.h>
//#include <PID_v1.h>
//#include "Motor_Library/Motor.h";
#include "Motor_Library/CtrlLoop.h";

int PWM = 4, InA1 = 5, InB1 = 6;
int PWM_val = 0;
Encoder myEnc(2, 3);
double Setpoint, Input, Output;
double Kp = 5, Ki = 0, Kd = 0;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
long oldPosition  = -999;
bool pidComputed = false;

Motor xMotor('d','d','d');
CtrlLoop xCtrlLoop('X', &myEnc, &xMotor, Kp, Ki, Kd);
void setup() {
  Serial.begin(115200);

  //TODO: Remove section since handled in constructor
//  pinMode(InA1, OUTPUT);
//  pinMode(InB1, OUTPUT);
//  pinMode(PWM, OUTPUT);

  //TODO: Erase line below
  //Input = myEnc.read();
  xCtrlLoop.updatePosition();

  //TODO: Erase line below since handled in constructor
  Setpoint = 0;

  //TODO: Erase linebelwo since handled in constructor
  //myPID.SetMode(AUTOMATIC);
  
  Serial.println("Ready");

}

void loop() {
  xCtrlLoop.updatePosition();
  //TODO: Erase section
//  long newPosition = myEnc.read();
//    if (newPosition != oldPosition) {
//      oldPosition = newPosition;
//      Input = newPosition;
//    }


  //TODO: Erase section
  xCtrlLoop.findMotorDirection();
//  if(Setpoint < Input)
//  {
//    myPID.SetControllerDirection(REVERSE);
//    Serial.print("Reverse controller mode");
//
//    //TODO: replace with goBackward function
//    digitalWrite(InA1, HIGH);
//    digitalWrite(InB1, LOW);
//
//    //xMotor.goForward();
//    
//    Serial.print("GO backward");
//  }
//  else 
//  {
//    myPID.SetControllerDirection(DIRECT);
//    Serial.print("DIRECT Controller Mode");
//
//    //TODO: replace with goBackward function
//    digitalWrite(InA1, LOW);
//    digitalWrite(InB1, HIGH);
//    Serial.print("Go forward");
//  }

  
  while (Serial.available() > 0) {
    Setpoint = Serial.parseInt();
  }
  
    //TODO: Erase line below
    //pidComputed = myPID.Compute();
    pidComputed = xCtrlLoop.pid->Compute();


    //TODO: Erase if block below
    if (Output > 0) {
      //digitalWrite(InB1, HIGH);
      //digitalWrite(InA1, LOW);
      //Serial.print("Go forward");
    }
    else if (Output < 0) {
      //digitalWrite(InB1, LOW);
      //digitalWrite(InA1, HIGH);
      //Serial.print("GO backward");
    }
    if(pidComputed) {
      //digitalWrite(InB1, LOW);
      //digitalWrite(InA1, LOW);
      //Serial.print("Brake");
    }

    //TODO: Remove this line since handled in MotorClass
    analogWrite(PWM, Output);


    //TODO: Debugging section should be made conditional
    Serial.print("SP: ");
    Serial.print(Setpoint);
    Serial.print(", In: ");
    Serial.print(Input);
    Serial.print(", Out: ");
    Serial.println(Output);
}

