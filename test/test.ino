#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4Motors motors;

#define echoPinR 0 // v
#define echoPinL 2 // v
#define echoPinF 14 // v
#define echoPinB 3 // v

#define trigPinR 1 // v
#define trigPinL 18 // v
#define trigPinF 13 // v
#define trigPinB 21 // v

const int baseSpeed = 100;

//PID constants
double kp = 1; // 0.1
double ki = 0.1; // 0.1
double kd = 1;   // 1
 
unsigned long currentTime, previousTime=0;
double elapsedTime;
double error;
double lastError = 0;
double input, output, setPoint;
double cumError=0, rateError;
int motorspeed;
 
void setup(){
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  //--------------------------
  pinMode(echoPinR,INPUT);
  pinMode(echoPinB,INPUT);
  pinMode(echoPinF,INPUT);
  pinMode(echoPinL,INPUT);        
  Serial.begin(9600);
  setPoint = 20;                          //set point at zero degrees
}    
 
void loop(){
  input = getDistance(1, 0);               //read from rotary encoder connected to A0
  //motorspeed = computePID(input);
  Serial.println(getDistance(trigPinL, echoPinL));
  //if (motorspeed > baseSpeed) motorspeed=baseSpeed;
  //if (motorspeed < -baseSpeed) motorspeed=-baseSpeed;
  //motors.setSpeeds(baseSpeed-motorspeed, baseSpeed+motorspeed);
}
 
double computePID(double inp){     
  //currentTime = millis();                //get current time
  //elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                                // determine error
  Serial.print("error: ");
  Serial.println(error);
  cumError += error / 1000; // * elapsedTime / 1000;                // compute integral
  Serial.print("cumError: ");
  Serial.println(cumError);
  rateError = (error - lastError); // elapsedTime;   // compute derivative
  Serial.print("rateError: ");
  Serial.println(rateError);
  double out = kp*error + ki*cumError + kd*rateError;                //PID output               

  lastError = error;                                //remember current error
  //previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
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