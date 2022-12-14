#include<Wire.h>
#include<Zumo32U4.h>
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
#include "TurnSensor.h"
const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;

//array for noice reduction. 
int arraySize = 10;
double array[10];



double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}

void turnToAngle(int target){// span {-180 -- 0 -- 180}
  while(true)
  {
  turnSensorUpdate();
  int currentAngle = getAngle();
    if(target != currentAngle && target < (currentAngle))
    { 
      motors.setSpeeds(200, 0);
      
      Serial.println((String)currentAngle);
    }
      else if(target != currentAngle && target > (currentAngle))
    {
      motors.setSpeeds(-200, 0);
      
      //Serial.println((String)currentAngle);
    }
    else if(-target == currentAngle || target == currentAngle)
    {
      motors.setSpeeds(0,0);
      break;
    }
  }
}

void setup() {
  turnSensorSetup();
  Serial.begin(9600);
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  //--------------------------
  pinMode(echoPinR, INPUT);
  pinMode(echoPinB, INPUT);
  pinMode(echoPinF, INPUT);
  pinMode(echoPinL, INPUT);
  //noiceReduction nR;
  //nR.resetArrays();
  
  
}    
 
int getAngle(){
  turnSensorUpdate();
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}


void loop() {
    if(getDistance(trigPinR,echoPinR)>50){
      delay(250);
      turnToAngle(-90);
      for(int i = 0; i<3; i++){
        buzzer.playFrequency(1200,1000,10);
        delay(2000);
      }
      motors.setSpeeds(121, 100);
      delay(2000);
      motors.setSpeeds(0, 0);
    }
  
    else
    {
      motors.setSpeeds(121,100);
    }
}