#include<Wire.h>
#include<Zumo32U4.h>
Zumo32U4Motors motors;
Zumo32U4IMU imu;
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
double setPoint = 0;
double cumError=0, rateError;
int motorspeed;
int currentAngle;
int baseSpeed = 221; // this it the left motorspeed with closest equality to the right 200 motor speed.
int regulatedSpeed = 200;
int status = 'f'; // 'f' for fine or follow wall
//PID constants
double kp = 8; // 0.1
double ki = 0.1; // 0.1
double kd = 2;   // 1
 
double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}

struct noiceReduction
{
  double getDistance(int trig, int echo) {
    int errorMargin = 30;
    int firstReading = 0;
    double sum = 0;
    double temp = 0;
    //Making a firstReading
    digitalWrite(trig, LOW); //clear
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); //Trigger pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW); //stop pulse
    firstReading = pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
    array[-1]=firstReading;
    sum=firstReading;
    //now sorting based on the first reading
    for (int i = 0; i < arraySize; i++){
      digitalWrite(trig, LOW); // clear
      delayMicroseconds(2);
      digitalWrite(trig, HIGH); // Trigger pulse
      delayMicroseconds(10);
      digitalWrite(trig, LOW); // stop pulse
      temp = pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
      if (array[i-1]<temp+errorMargin && array[i-1] > temp-errorMargin) 
      // so here we only count measurements, thats 30(errorMargin) +- from the last measurement. 
      {
        array[i]=temp;
        sum += array[i];
        delay(5);//10
      }
      else{ // if the readings are not within the errormargin.
        resetArrays();
        array[-1]=temp;
      }
    }
  return sum/arraySize;// Return average distance
  }

  void resetArrays(){
    for (int i = 0; i < arraySize; i++) //size of divided by 8 cuz its a double
    {
        array[i]=0;
    }
    
  }
};

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

double computePID(double inp){     
  //currentTime = millis();                //get current time
  //elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;      // determine error
  Serial.print("error: ");
  Serial.println(error);
  //cumError += error / 1000;   // compute integral
  Serial.print("cumError: ");
  Serial.println(cumError);
  rateError = (error - lastError);   // compute derivative
  Serial.print("rateError: ");
  Serial.println(rateError);
  double out = kp*error /*+ ki*cumError*/ + kd*rateError;                //PID output               

  lastError = error;                                //remember current error
  

  return out;                                        //have function return the PID output
}

int output;
char followWall(){
   //before follow Wall the robot must be on a 90 degree angle on the wall with a 0 calibration on the gyro. 
    while(getDistance(trigPinF,echoPinF)>20){
        if(getDistance(trigPinR,echoPinR) < 19 || getDistance(trigPinR,echoPinR) > 21){
          regulatedSpeed = 200;
          if(getDistance(trigPinR,echoPinR) < 19){
            baseSpeed -= 2;
            delay(20);  
          }
          else if(getDistance(trigPinR,echoPinR) > 21 && getDistance(trigPinR,echoPinR) < 50){
            baseSpeed += 2;
            delay(20);
          }
          else if(getDistance(trigPinR,echoPinR) >= 50){
            motors.setSpeeds(0, 0);
            return 'c'; // 'c' for corner
          }
        }
        else{
          baseSpeed = 221;
          motors.setSpeeds(baseSpeed,regulatedSpeed);
          currentAngle = getAngle();
          output = computePID(currentAngle);
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
  motors.setSpeeds(0, 0);
}
else if (status == 'o'){
  motors.setSpeeds(0, 0);
  turnToAngle(90-2);
  status = 'A';
}


}

