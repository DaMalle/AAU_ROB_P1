#include<Wire.h>
#include<Zumo32U4.h>

int maxSpeedR = 200;
int maxSpeedL = 221;
Zumo32U4Motors motors;

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
  keepDistance();
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

void keepDistance() {
  int distance = getDistance(trigPinL, echoPinL);
  motors.setSpeeds(maxSpeedL, maxSpeedR);
  if(distance <= 50 && distance >= 21){
    motors.setSpeeds(maxSpeedL/2, maxSpeedR/2);
  }
  else if(distance <= 20){
    motors.setSpeeds(0,0);
  }
} 