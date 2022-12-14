#include<Wire.h>
#include<Zumo32U4.h>
#include<Zumo32U4Motors.h>
#include<Zumo32U4Encoders.h>

Zumo32U4Motors motors;

int maxSpeedR = 200;
int maxSpeedL = 221;

const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;
 
void setup() {
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
}    
 
void loop() {
  followAlong();
}

double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}

void followAlong() {
  int distance = getDistance(trigPinB,echoPinB); // Define local variable 'distance' by using the distance from the backwards facing sensor
  int followSpeedR = maxSpeedR - distance*3; // Local variable 'followSpeedR' is defined as the max speed on the right subtracted by the sensors distance multiplied by 3
  int followSpeedL = maxSpeedL - distance*3; 
  followSpeedR = constrain(followSpeedR,0,200); // Here 'followSpeedR' is being constrained to not go below 0 and not be above 200 in speed
  followSpeedL = constrain(followSpeedL,0,221);
  motors.setSpeeds(followSpeedL,followSpeedR); // This is where the speeds for both the motors is set to their own individual value since the motors are not that accurate.
}