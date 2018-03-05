//This code mainly shows how to split the string from pc input into different strings to be used by control loop and motor.
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


String PC_input, command, id, value;

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
    Serial.print("Go backward");
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
    PC_input = Serial.readStringUntil('\n');
    //splitting PC_input:
    command = getValue(PC_input, ',', 0);
    id = getValue(PC_input, ',', 1);
    value = getValue(PC_input, ',', 2);

    Setpoint = value.toDouble();
    //
    //Input for motor:
    
    
//    Setpoint = Serial.parseInt();
//    Serial.print("New setpoint:" + Setpoint);
  }
      Serial.print(command + ' ' + id + ' ' + value);

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
String getValue(String data, char separator, int index)
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
