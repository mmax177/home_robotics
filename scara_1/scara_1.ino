#include "DualVNH5019MotorShield.h"
#include "math.h"

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

///////////////////////////////////////////////////////
/////                  My Program                 /////
///////////////////////////////////////////////////////

//constant
double l1 = 72;
double l2 = 132;
double d = 128;

const double pi = atan(1)*4;

//initial variable
double c,e,delta,psi,gamma,epsilon,alpha,beta;
double dx,dy,dc,de,ddelta,dpsi,dgamma,depsilon,dalpha,dbeta;

double x = d/2;                        //assume apha = beta = pi/2
double y = l1 + sqrt(l2*l2 - d*d/4);


void loop() {
  // read the analog in value:
  sensorValue0 = analogRead(analogInPin0);
  sensorValue1 = analogRead(analogInPin1);
  // map it to the range of the analog out:
  outputValue0 = map(sensorValue0, 0, 1023, -400, 400)+63;
  outputValue1 = map(sensorValue1, 0, 1023, -400, 400)+72;
  if (outputValue0 < 2 && outputValue0 > -2)outputValue0 = 0;
  if (outputValue1 < 2 && outputValue1 > -2)outputValue1 = 0;
  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue0);

  ////////////////////// compute ////////////////////////
  //position
  c = sqrt(x*x + y*y);
  e = sqrt((d-x)*(d-x) + y*y);
  delta = atan(y/x);
  psi = atan(y/(d-x));

  gamma = acos((c*c - l2*l2 + l1*l1)/(2*l1*c));
  epsilon = acos((e*e - l2*l2 + l1*l1)/(2*l1*e));

  alpha = delta + gamma;
  beta = pi - psi - epsilon;

  //velocity
  dx = outputValue0;
  dy = outputValue1;

  dc = (x*dx + y*dy)/c;
  de = ((x-d)*dx + y*dy)/e;

  ddelta = (x*dy - y*dx)*(cos(delta)*cos(delta))/(x*x);
  dpsi = ((d-x)*dy + y*dx)*(cos(psi)*cos(psi))/((d-x)*(d-x));

  dgamma = (l2*l2 - l1*l1 - c*c)*dc/(2*l1*c*c*sin(gamma));
  depsilon = (l2*l2 - l1*l1 - e*e)*de/(2*l1*e*e*sin(epsilon));

  dalpha = ddelta + dgamma;
  dbeta = -dpsi -depsilon;

  //md.setM1Speed((int)(dalpha));
  //md.setM2Speed((int)(dbeta));
  md.setSpeeds((int)(dalpha), (int)(dbeta*50));
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
  /*
  Serial.print("position = ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print("\tvelocity = ");
  Serial.print(dx);
  Serial.print(",");
  Serial.println(dy);
  */
  Serial.print("angle = ");
  Serial.print(alpha);
  Serial.print(",");
  Serial.print(beta);
  Serial.print("\tomega = ");
  Serial.print(dalpha);
  Serial.print(",");
  Serial.println(dbeta);
  
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
