#include <Wire.h>
#include <Zumo32U4.h>
#include <Math.h>

const int echoPinR = 21;
const int echoPinL = 18;
const int echoPinF = 4;
const int echoPinB = 20;

const int trigPinR = 0;
const int trigPinL = 17;
const int trigPinF = 1;
const int trigPinB = 30;

Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;

#include "TurnSensor.h"

void setup() 
{
  Serial.begin(9600);
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  
  pinMode(echoPinR,INPUT);
  pinMode(echoPinB,INPUT);
  pinMode(echoPinF,INPUT);
  pinMode(echoPinL,INPUT); 

  turnSensorSetup();
  turnSensorReset();
  encoders.init();
}

void loop() 
{
  GetDistanceToObject();
}

// Distance sensor functions
double getDistance(const int trig, const int echo) 
{
    digitalWrite(trig, LOW); // clear
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); // Trigger pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW); // stop pulse

    // Return distance
    return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}

// Gyroscope functions
void rotateToTarget(int target) {  // span {-180 -- 0 -- 180}
  int currentAngle = getAngle(); 

  while(-target != currentAngle && target != currentAngle) {
    currentAngle = getAngle();

    if(target != currentAngle && target < (currentAngle)) motors.setSpeeds(100, -100);
    else if(target != currentAngle && target > (currentAngle)) motors.setSpeeds(-100, 100);
  }
  motors.setSpeeds(0,0);
}

int getAngle() {
  turnSensorUpdate();
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

//This is the function for avoiding objects
void GetDistanceToObject()
{
  int StraightDistance = getDistance(trigPinF,echoPinF);
  int LastDistance = StraightDistance;
  int LastDistanceUse = 0;
  int ErrorMargain = 50;
  int NewAngle = 2;
  int DriftCalibration = 0;
  while(LastDistance - StraightDistance < ErrorMargain)
  {
    LastDistanceUse = LastDistance;
    delay(250);
    rotateToTarget(NewAngle);
    LastDistance = getDistance(trigPinF,echoPinF);
    NewAngle = NewAngle + 1;
    DriftCalibration = DriftCalibration + 1;
  }
  double LengthA = sqrt(sq(LastDistanceUse) - sq(StraightDistance));

  rotateToTarget(90-DriftCalibration);
  turnSensorReset();

  int DelayLength = LengthA * 125;
  motors.setSpeeds(100, 79);
  delay(DelayLength*2);
  motors.setSpeeds(0,0);
  while(getAngle() > -90)
  {
    motors.setSpeeds(100, -21);
  }
  while(getDistance(trigPinR,echoPinR) > 25)
  {
    motors.setSpeeds(100,79);
  }
  while(getDistance(trigPinR,echoPinR) < 25)
  {
    motors.setSpeeds(100,79);
  }
  delay(DelayLength*2);
  while(getAngle() > -179)
  {
    motors.setSpeeds(100, -21);
  }
  motors.setSpeeds(0,0);
}