#include<Wire.h>
#include<Zumo32U4.h>
#include<math.h>
#include<BasicLinearAlgebra.h>
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
#include "TurnSensor.h"
const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;

//array for noice reduction. 
int arraySize = 10;
double array[10];
//variables
double error;
double lastError = 0;

double cumError=0, rateError;
int motorspeed;
int currentAngle;
int regulatedSpeedL = 221; // this it the left motorspeed with closest equality to the right 200 motor speed.
int baseSpeedL = 221;
int baseSpeedR = 200;
int regulatedSpeedR = 200;
int status = 'f'; // 'f' for fine or follow wall 'c' for corner 'o' for obstacle

int Angle;
double getDistance(int trig, int echo) {
  Angle= getAngle();
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  if(trig == trigPinR){
  double dis = pulseIn(echo, HIGH) * 0.034029 /2; // Speed of sound wave divided by 2 (go and back)
  ////Serial.println(dis);
  ////Serial.print("Angle: ");
  ////Serial.println(Angle);
  double result = (cos(Angle *M_PI /180)*dis);
  ////Serial.println(result);
  return result;
  }
  else return pulseIn(echo, HIGH) * 0.034029 /2; // Speed of sound wave divided by 2 (go and back)
}



void setup() {
  turnSensorSetup();
  //Serial.begin(9600);
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
  turnSensorReset();
  motors.setSpeeds(221, 200);
  
}    
 
int32_t getAngle(void) {
    turnSensorUpdate();
    return (((int32_t)turnAngle >> 16) * 360) >> 16;
}


double computePID(int setPoint,double input,float kp, float ki, float kd){     
  error = setPoint - input;      // determine error
  ////Serial.print("error: ");
  ////Serial.println(error);
  cumError += error;   // compute integral
  ////Serial.print("cumError: ");
  ////Serial.println(cumError);
  rateError = (error - lastError);   // compute derivative
  ////Serial.print("rateError: ");
  ////Serial.println(rateError);
  double out = kp*error + ki*cumError + kd*rateError;                //PID output               

  lastError = error;                                //remember current error
  return out;                                        //have function return the PID output
}
float localError = 0;
int output;
char followWall(){
  while(getDistance(trigPinF,echoPinF)>15){
    //keepDistance();
    //Serial.println(getDistance(trigPinR,echoPinR));
    //regulatedSpeedR = baseSpeedR;
    double right = getDistance(trigPinR,echoPinR);
    int change = computePID(20,right,4,0,1); // P4 , I0 , D1
    regulatedSpeedR = baseSpeedR + change;
    //Serial.println(regulatedSpeedR);
    motors.setSpeeds(baseSpeedL-change, regulatedSpeedR);         
    if(getDistance(trigPinR,echoPinR) >= 50){
      motors.setSpeeds(0, 0); 
      return 'c'; // 'c' for corner
    }
  }
  motors.setSpeeds(0, 0);
  return 'o'; // 'o' for obstacle
}

void corner(){
  if (getDistance(trigPinF,echoPinF)<20) motors.setSpeeds(0,0);
  else{
    buzzer.playFrequency(1200,1000,15);
    turnSensorReset();
    motors.setSpeeds(250, 150);
    while(getAngle() != -90+1);
    motors.setSpeeds(0,0);
    turnSensorReset();
    motors.setSpeeds(221,200);
    delay(300);
    motors.setSpeeds(0,0);
    
    status = 'f';
  }
}

void keepDistance() {
  int distance = getDistance(trigPinL, echoPinL);
  if(distance <= 50 && distance >= 21){
    baseSpeedL = baseSpeedL-100;
    baseSpeedR = baseSpeedR-100;
  }
  else if(distance <= 20){
    motors.setSpeeds(0,0);
    delay(1000);
  }
  else baseSpeedL = 221; baseSpeedR = 200;
}

void loop() {
switch (status){
  case 'f': status = followWall(); break;
  case 'c':corner(); break;
  case 'o':
  buzzer.playFrequency(1200,1000,15);
  motors.setSpeeds(0, 0);
  status = 'A';
  break;
}
}


