#include <Wire.h>
#include <Zumo32U4.h>

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
}

void loop() 
{
  rotateToTarget(90);  
  while(getDistance(trigPinR,echoPinR) <= 25) //Error Margin of 5 centimeters, as target is 20cm away
  {
    Serial.print((String)getDistance(trigPinR,echoPinR));
    //Drive Forwards Code
    motors.setSpeeds(100, 100);
  }
  delay(500);
  motors.setSpeeds(0,0);
  rotateToTarget(0);
  //Drive forwards code
  motors.setSpeeds(100, 100);
  delay(1000);
  while(getDistance(trigPinR,echoPinR) < 20)
  {
    //Drive forwards code
    motors.setSpeeds(100, 100);
  }
  delay(500);
  motors.setSpeeds(0, 0);
  rotateToTarget(-90);

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
    turnSensorUpdate();
    currentAngle = getAngle();

    if(target != currentAngle && target < (currentAngle)) motors.setSpeeds(100, -100);
    else if(target != currentAngle && target > (currentAngle)) motors.setSpeeds(-100, 100);
  }
  motors.setSpeeds(0,0);
}

int getAngle() {
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}