#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
}

const int analogInPin0 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin1 = A1;
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue0 = 0;        // value read from the pot
int sensorValue1 = 0;
int outputValue0 = 0;        // value output to the PWM (analog out)
int outputValue1 = 0;

void loop() {
  // read the analog in value:
  sensorValue0 = analogRead(analogInPin0);
  sensorValue1 = analogRead(analogInPin1);
  // map it to the range of the analog out:
  outputValue0 = map(sensorValue0, 0, 1023, -400, 400)+63;
  outputValue1 = map(sensorValue1, 0, 1023, -400, 400)+72;
  // change the analog out value:
  analogWrite(analogOutPin, outputValue0);

  md.setM1Speed(outputValue0);
  md.setM2Speed(outputValue1);
  stopIfFault();

  // print the results to the serial monitor:
  Serial.print("sensor = " );
  Serial.print(sensorValue0);
  Serial.print(",");
  Serial.print(sensorValue1);
  Serial.print("\t output = ");
  Serial.println(outputValue0);
  Serial.print(",");
  Serial.println(outputValue1);
  
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
