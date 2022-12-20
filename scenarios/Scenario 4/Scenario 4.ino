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
  driveAroundObject();
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
void driveAroundObject()
{
  int StraightDistance = getDistance(trigPinF,echoPinF); //Distance to the object
  int LastDistance = StraightDistance; //This is just init for LastDistance, it's set to StraightDistance for calculation purposes
  int LastDistanceUse = 0;
  int ErrorMargain = 50;
  int NewAngle = 1; //Information for the Gyroscope
  int DriftCalibration = 0;
  while(LastDistance - StraightDistance < ErrorMargain) //When the Distance overshoots, as the object can no longer be seen, this will no longer amount to true
  {
    LastDistanceUse = LastDistance; //Saving prior distance, as we would otherwise get overshot length
    delay(250);
    rotateToTarget(NewAngle); //Gyroscope used to turn slowly away from target
    LastDistance = getDistance(trigPinF,echoPinF);
    NewAngle = NewAngle + 1;
    DriftCalibration = DriftCalibration + 1;
  }
  double LengthA = sqrt(sq(LastDistanceUse) - sq(StraightDistance)); //Using Pythagoras' theorem to calculate A (Width of the object)

  rotateToTarget(90-DriftCalibration); //Makeshift gyrocalibration trying to account for drift.
  turnSensorReset();

  int DelayLength = LengthA * 125; //A delay of 125 on speeds (100,79) is 1cm of movement from the Zumo32U4, so this delay * Width will make the zumo drive the width.
  motors.setSpeeds(100, 79);
  delay(DelayLength*2); //We want our robot to be double the length of the object away, before turning
  motors.setSpeeds(0,0);
  while(getAngle() > -90) //As the angle is currently 0, we use getAngle() to set a target for -90 (A turn to the right)
  {
    motors.setSpeeds(100, -21); //Code to turn in a parabola towards the right
  }
  //This is hardcoded to get the Zumo past the object
  while(getDistance(trigPinR,echoPinR) > 25)
  {
    motors.setSpeeds(100,79);
  }
  while(getDistance(trigPinR,echoPinR) < 25)
  {
    motors.setSpeeds(100,79);
  }

  delay(DelayLength*2); //Drive past the object, so it does not get hit on turn in
  while(getAngle() > -179) // New target, current angle is -90
  {
    motors.setSpeeds(100, -21);
  }
  motors.setSpeeds(0,0);
}
//The Zumo has now passed the object