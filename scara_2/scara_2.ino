#include "DualVNH5019MotorShield.h"
#include "math.h"
#include <Encoder.h>

#define PI 3.14159265359
#define GEAR_RATIO 159
#define PULSE_PER_ROUND 2000
#define RAD_TO_PULSE GEAR_RATIO*PULSE_PER_ROUND/PI
#define RAD_PER_SECVOLT 456/60

DualVNH5019MotorShield md;
Encoder encoder1(2, 3);
Encoder encoder2(4, 5);

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

int sensorValue0 = 0;        // value read from the pot
int sensorValue1 = 0;
int outputValue0 = 0;        // value output to the PWM (analog out)
int outputValue1 = 0;

///////////////////////////////////////////////////////
/////                  My Program                 /////
///////////////////////////////////////////////////////

//constant
float l1 = 72;
float l2 = 132;
float d = 128;

//initial variable
float c,e,delta,psi,gamma,epsilon;
float dx,dy,dc,de,ddelta,dpsi,dgamma,depsilon,dalpha,dbeta;

//assume alpha = beta = pi/2
float alpha = 90;
float beta = 90;

float x = d/2;
float y = l1 + sqrt(l2*l2 - d*d/4);

int alpha_speed,beta_speed;
int prev_millis = 0;
int this_millis = 0;

float kp = 6.0;
float kd = 1.3;

long alpha_this_error = 0;
long beta_this_error = 0;

long alpha_prev_error = 0;
long beta_prev_error = 0;

long alpha_pulse = 0;
long beta_pulse = 0;
unsigned char alpha_result,beta_result;

void loop() {
  //////////////////////// millis ///////////////////////////////
  prev_millis = this_millis;
  this_millis = millis();
  //////////////////////// analog ///////////////////////////////
  // read the analog in value:
  sensorValue0 = analogRead(analogInPin0);
  sensorValue1 = analogRead(analogInPin1);
  
  // map it to the range of the analog out:
  outputValue0 = map(sensorValue0, 0, 1023, -400, 400)+63;
  outputValue1 = map(sensorValue1, 0, 1023, -400, 400)+72;
  
  //filter
  if (outputValue0 < 2 && outputValue0 > -2)outputValue0 = 0;
  if (outputValue1 < 2 && outputValue1 > -2)outputValue1 = 0;

  ////////////////////// compute ////////////////////////
  if(Serial.available()) {
    x = Serial.parseFloat();
    y = Serial.parseFloat();
    
    Serial.print(x);
    Serial.print("\t");
    Serial.println(y);
  }

  x += outputValue0*(this_millis - prev_millis)/10000;
  y += outputValue1*(this_millis - prev_millis)/10000;
  //position
  c = sqrt(x*x + y*y);
  e = sqrt((d-x)*(d-x) + y*y);
  
  delta = atan(y/x);
  psi = atan(y/(d-x));

  gamma = acos((c*c - l2*l2 + l1*l1)/(2*l1*c));
  epsilon = acos((e*e - l2*l2 + l1*l1)/(2*l1*e));

  alpha = delta + gamma;
  beta = PI - psi - epsilon;

  ///////////////////////// encoder /////////////////////////
  alpha_result = encoder1.read();
  beta_result = encoder2.read();
  
  ///////////////////////// PD //////////////////////////////
  alpha_this_error = alpha*RAD_TO_PULSE - alpha_pulse;
  alpha_speed = kp * alpha_this_error + kd * (alpha_this_error - alpha_prev_error);
  alpha_prev_error = alpha_this_error;
  alpha_speed = map(alpha_speed/RAD_TO_PULSE/RAD_PER_SECVOLT,-12,12,-400,400);

  beta_this_error = beta*RAD_TO_PULSE - beta_pulse;
  beta_speed = kp * beta_this_error + kd * (beta_this_error - beta_prev_error);
  beta_prev_error = beta_this_error;
  beta_speed = map(beta_speed/RAD_TO_PULSE/RAD_PER_SECVOLT,-12,12,-400,400);
  /////////////////////////// Motor /////////////////////////////
  md.setSpeeds((int)(alpha_speed), (int)(beta_speed));
  stopIfFault();

  ///////////////////////// Serial ///////////////////////////
  
  //analog
  Serial.print("sensor = " );
  Serial.print(sensorValue0);
  Serial.print(",");
  Serial.print(sensorValue1);
  Serial.print("\t output = ");
  Serial.println(outputValue0);
  Serial.print(",");
  Serial.println(outputValue1);
  

  Serial.print("position = ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print("\tvelocity = ");
  Serial.print(dx);
  Serial.print(",");
  Serial.println(dy);
  
  
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
  //delay(2);
}
