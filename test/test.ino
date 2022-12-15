#include<Wire.h>
#include<Zumo32U4.h>

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
  Serial.println(getDistance(trigPinF, echoPinF));
}

double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) * 0.034029 / 2; // Speed of sound wave divided by 2 (go and back)
}