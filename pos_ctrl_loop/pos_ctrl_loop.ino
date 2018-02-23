#include <Encoder.h>
#include <PID_v1.h>

int PWM = 4, InA1 = 5, InB1 = 6;
int PWM_val = 0;
Encoder myEnc(2, 3);
double Setpoint, Input, Output;
double Kp = 5, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
long oldPosition  = -999;
bool pidComputed = false;

void setup() {
  Serial.begin(115200);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM, OUTPUT);
  Input = myEnc.read();
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  Serial.println("Ready");

}

void loop() {
  long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      Input = newPosition;
    }
  if(Setpoint < Input)
  {
    myPID.SetControllerDirection(REVERSE);
    Serial.print("Reverse controller mode");
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    Serial.print("GO backward");
  }
  else 
  {
    myPID.SetControllerDirection(DIRECT);
    Serial.print("DIRECT Controller Mode");
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    Serial.print("Go forward");
  }
  while (Serial.available() > 0) {
    Setpoint = Serial.parseInt();
  }
    pidComputed = myPID.Compute();

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
    analogWrite(PWM, Output);
    
    Serial.print("SP: ");
    Serial.print(Setpoint);
    Serial.print(", In: ");
    Serial.print(Input);
    Serial.print(", Out: ");
    Serial.println(Output);
}

