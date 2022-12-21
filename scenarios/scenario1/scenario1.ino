#include<Wire.h>
#include<Zumo32U4.h>
#include<BasicLinearAlgebra.h>
#include<math.h>
#include <VL53L0X.h>

VL53L0X tof;

Zumo32U4IMU imu;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;

#include"TurnSensor.h"

const int trigPinF = 1, trigPinB = 30, trigPinL = 17,
          echoPinF = 4, echoPinB = 20, echoPinL = 18;

//array for noice reduction. 
int arraySize = 10;
double array[10];
//variables
double error;
double lastError = 0;
double currentTime = millis();
double previousTime = 0;
double elapsedTime;

double cumError=0, rateError;
int motorspeed;
int currentAngle;
int regulatedSpeedL = 208; // this it the left motorspeed with closest equality to the right 200 motor speed.
int baseSpeedL = 208;
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
  return pulseIn(echo, HIGH) * 0.034029 /2; // Speed of sound wave divided by 2 (go and back)
}

void setup(void) {
    //Serial.begin(9600);
    Wire.begin();

    pinMode(trigPinB, OUTPUT);
    pinMode(trigPinF, OUTPUT);
    pinMode(trigPinL, OUTPUT);

    pinMode(echoPinB, INPUT);
    pinMode(echoPinF, INPUT);
    pinMode(echoPinL, INPUT);

    tof.init();
    tof.setTimeout(500);
    tof.setMeasurementTimingBudget(100000);

    delay(1000);
    turnSensorSetup();
    turnSensorReset();

    calibrateWall(); 
}

void calibrateWall() {
    const int dataPointsTotal = 7;
    int angle;
    double length;
    int arr[dataPointsTotal] = {0, 5, 10, 15, -5, -10, -15};

    BLA::Matrix<dataPointsTotal, 2> X;
    BLA::Matrix<dataPointsTotal, 1> b;
      
    for(int i=0; i<dataPointsTotal; i++) {
        rotateToTarget(arr[i]);
        angle = getAngle();
        length = tof.readRangeSingleMillimeters() * 0.1;
    
        Serial.println("angle: " + (String) angle + " Lenght: " + (String) length);

        X(i, 0) = 1;
        X(i, 1) = length * cos(angle * M_PI / 180);
        b(i, 0) = length * sin(angle * M_PI / 180);

        delay(50);
    }

    BLA::Matrix<2, dataPointsTotal> X_transpose = ~X;
    BLA::Matrix<2, 2> inversed_X_transpose_dot_X = X_transpose * X;
    BLA::Invert(inversed_X_transpose_dot_X);

    BLA::Matrix<2, 1> x_hat = inversed_X_transpose_dot_X * X_transpose * b;
    
    // intersection between wall-equation and perpendicular line.
    float x = (-x_hat(1, 0) * x_hat(0, 0) / (x_hat(1, 0) * x_hat(1, 0) + 1));
    float y = x_hat(0, 0) / (x_hat(1, 0) * x_hat(1, 0) + 1);

    Serial.println(round(atan2(y, x) * 180 / M_PI));    
    rotateToTarget(round(atan2(y, x) * 180 / M_PI));
}

int32_t getAngle(void) {
    turnSensorUpdate();
    return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

double getUltraSonicSensorData(const int trig, const int echo) {
    digitalWrite(trig, LOW); // clear pin
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); // send pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    // speed of sound divided by 2.
    return pulseIn(echo, HIGH) * 0.034029 / 2; // distance
}

void rotateToTarget(const int target) {  // span {-180 -- 0 -- 180}
    const int turnSpeed = 75;
    int currentAngle = getAngle();
    Serial.println("current angle: " + (String) currentAngle);

    while(target != currentAngle) {
        currentAngle = getAngle();
        if(target < currentAngle) motors.setSpeeds(turnSpeed, -turnSpeed);
        else if(target > (currentAngle)) motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    motors.setSpeeds(0,0);
}

double computePID(int setPoint, double input, float kp, float ki, float kd){ 
  currentTime = millis();  
  elapsedTime = currentTime - previousTime;
  //Serial.println("elapsedTime: " + (String) elapsedTime);
  error = setPoint - input;      // determine error
  cumError += error * elapsedTime;   // compute integral
  rateError = (error - lastError) / elapsedTime;   // compute derivative
  double out = kp*error + ki*cumError + kd*rateError;                //PID output               
  //Serial.println("error: " + (String) error);
  //Serial.println("cumError: " + (String) cumError);
  //Serial.println("rateError: " + (String) rateError);
  //lastError = error;                                //remember current error
  previousTime = currentTime;
  return out;                                        //have function return the PID output
}

float localError = 0;
int output;
char followWall(){
  while(getDistance(trigPinF,echoPinF)>15) {
    double inputVar = cos(getAngle() * M_PI / 180) * (tof.readRangeSingleMillimeters() * 0.1);
    if(inputVar >= 50) {
      motors.setSpeeds(0, 0); 
      return 'c'; // 'c' for corner
    }
    else {
      int change = computePID(30, inputVar, 0.5 , 0, 0); // P1.5 , I0.00004 , D0.000001
      if (change > 200) change = 200;
      //Serial.println("right: " + (String) right);
      //Serial.println("change: " + (String) change);    
      motors.setSpeeds(baseSpeedL-change, baseSpeedR + change); 
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
    motors.setSpeeds(208,200);
    delay(1000);
    calibrateWall();
    turnSensorReset();
    cumError=0;

    
    status = 'f';
  }
}

void loop() {
  followWall();
  switch (status){
    case 'f': status = followWall(); break;
    case 'c': corner(); break;
    case 'o':
    buzzer.playFrequency(1200,1000,15);
    motors.setSpeeds(0, 0);
    status = 'A';
    break;
  }
  //motors.setSpeeds(208, 200);
}