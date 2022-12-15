#include<Wire.h>
#include<Zumo32U4.h>
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
unsigned long currentTime, previousTime=0;
double elapsedTime;
double error;
double lastError = 0;

double cumError=0, rateError;
int motorspeed;
int currentAngle;
int baseSpeed = 221; // this it the left motorspeed with closest equality to the right 200 motor speed.
int regulatedSpeed = 200;
int status = 'f'; // 'f' for fine or follow wall 'c' for corner 'o' for obstacle

 
double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) * 0.034029 /2; // Speed of sound wave divided by 2 (go and back)
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
  turnSensorReset();
  motors.setSpeeds(221, 200);
  
}    
 
int getAngle(){
  turnSensorUpdate();
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

double computePID(int setPoint,double input,float kp, float ki, float kd){     
  //currentTime = millis();                //get current time
  //elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - input;      // determine error
  Serial.print("error: ");
  Serial.println(error);
  cumError += error;   // compute integral
  Serial.print("cumError: ");
  Serial.println(cumError);
  rateError = (error - lastError);   // compute derivative
  Serial.print("rateError: ");
  Serial.println(rateError);
  double out = kp*error + ki*cumError + kd*rateError;                //PID output               

  lastError = error;                                //remember current error
  

  return out;                                        //have function return the PID output
}
float localError = 0;
int output;
char followWall(){
   //before follow Wall the robot must be on a 90 degree angle on the wall with a 0 calibration on the gyro. 
    while(getDistance(trigPinF,echoPinF)>15){
      if(getDistance(trigPinR,echoPinR) < 18 || getDistance(trigPinR,echoPinR) > 22){
        regulatedSpeed = 200;
        float right = getDistance(trigPinR,echoPinR);
        baseSpeed = 221 + computePID(20,right,2,0,0);         
        if(getDistance(trigPinR,echoPinR) >= 50){
          motors.setSpeeds(0, 0); 
          return 'c'; // 'c' for corner
        }
      }
      else{
        baseSpeed = 221;
        motors.setSpeeds(baseSpeed,regulatedSpeed);
        currentAngle = getAngle();
        output = computePID(0,currentAngle,10,0.2,0);
        regulatedSpeed = 200 + output;
        Serial.println(regulatedSpeed);
        Serial.println(currentAngle);
      }  
    }
    motors.setSpeeds(0, 0);
    return 'o'; // 'o' for obstacle
}

void loop() {
if(status == 'f'){
  status = followWall();  
}
else if (status == 'c'){
  buzzer.playFrequency(1200,1000,15);
  motors.setSpeeds(250, 150);
  delay(1500);
  motors.setSpeeds(221, 200);
  delay(700);
  status = 'f';
}
else if (status == 'o'){
  motors.setSpeeds(0, 0);
  status = 'A';
}
}

