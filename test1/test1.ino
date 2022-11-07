
// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //
#include<Zumo32U4.h>
Zumo32U4OLED lcd;
Zumo32U4ProximitySensors dis;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

#define echoPinR 0
#define echoPinB 14


#define trigPinR 1
#define trigPinB 13
#define ms 2
#define l_ms 10 

// defines variables
long durationF,durationL,durationR,durationB; // variable for the duration of sound wave travel
int distanceF,distanceL,distanceR,distanceB; // variable for the distance measurement

void setup() {
 
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  
 
  pinMode(echoPinR,INPUT);
 
  pinMode(echoPinB,INPUT);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  dis.initThreeSensors();
  motors.setRightSpeed(100);
  motors.setLeftSpeed(100);
}
void loop() {

  digitalWrite(trigPinR, LOW);
  delayMicroseconds(ms);
  digitalWrite(trigPinR, HIGH); // Trigger another pulse
  delayMicroseconds(l_ms);
  digitalWrite(trigPinR, LOW);
  // Calculating the distance
  durationR = pulseIn(echoPinR, HIGH);
  distanceR = durationR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

delayMicroseconds(50);

  digitalWrite(trigPinB, LOW);
  delayMicroseconds(ms);
  digitalWrite(trigPinB, HIGH); // Trigger another pulse
  delayMicroseconds(l_ms);
  digitalWrite(trigPinB, LOW);
  // Calculating the distance
  durationB = pulseIn(echoPinB, HIGH);
  distanceB = durationB * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

delayMicroseconds(50);

  // Displays the distance on the Serial Monitor

  Serial.print("DistanceR: ");
  Serial.print(distanceR);
  Serial.println(" cm");
  Serial.print("DistanceB: ");
  Serial.print(distanceB);
  Serial.println(" cm");
  if (distanceR > 20 && distanceR < 25){
    motors.setSpeeds(110,100);
  }
  else if(distanceR > 25 && distanceR < 30){
    motors.setSpeeds(120,100);
  }
  else if(distanceR > 30 && distanceR < 40){
    motors.setSpeeds(150,100);
  }
  else if (distanceR < 15 && distanceR > 10){
    motors.setSpeeds(100,110);
  }
  else if (distanceR < 10 && distanceR > 7){
    motors.setSpeeds(100,130);
  }
    else if (distanceR < 7 && distanceR > 0){
    motors.setSpeeds(100,125);
  }
  else if (distanceR >16 && distanceR < 19){
    motors.setSpeeds(100,100);
  }
}

