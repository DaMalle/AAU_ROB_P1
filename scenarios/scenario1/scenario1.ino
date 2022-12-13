#include<Wire.h>
#include<Zumo32U4.h>
#include<BasicLinearAlgebra.h>
#include<math.h>

Zumo32U4IMU imu;
Zumo32U4Motors motors;

#include"TurnSensor.h"

const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;

int arraySize = 10;
double array[10];

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


void setup(void) {
    Serial.begin(9600);

    pinMode(trigPinR, OUTPUT);
    pinMode(trigPinB, OUTPUT);
    pinMode(trigPinF, OUTPUT);
    pinMode(trigPinL, OUTPUT);

    pinMode(echoPinR, INPUT);
    pinMode(echoPinB, INPUT);
    pinMode(echoPinF, INPUT);
    pinMode(echoPinL, INPUT);

    delay(1000);
    turnSensorSetup();
    turnSensorReset();

    calibrateWall(trigPinF, echoPinF);
}

void loop(void) {
    //Serial.println(getGyroAngle());
    //delay(1000);
}

void calibrateWall(const int trig, const int echo) {
    const int dataPointsTotal = 20, turnSpeed = 100;
    int angle;
    double length;

    noiceReduction nR;
    nR.resetArrays();

    BLA::Matrix<dataPointsTotal, 2> X;
    BLA::Matrix<dataPointsTotal, 1> b;
      
    motors.setSpeeds(-turnSpeed, turnSpeed);
    for(int i=0; i<dataPointsTotal; i++) {
        angle = getGyroAngle();
        length = nR.getDistance(trig, echo);

        X(i, 0) = 1;
        X(i, 1) = length * cos(angle * M_PI / 180);
        b(i, 0) = length * sin(angle * M_PI / 180);
        Serial.print(length);
        Serial.print(" ");
        Serial.print(angle);
        Serial.print(" ");
        Serial.print(length * cos(angle * M_PI / 180));
        Serial.print(" ");
        Serial.println(length * sin(angle * M_PI / 180));
        if (i < dataPointsTotal / 2) rotateToTarget(i);
        else rotateToTarget((dataPointsTotal / 2 - i-1));
        delay(50);
    }
    motors.setSpeeds(0, 0);
    
    BLA::Matrix<2, dataPointsTotal> X_transpose = ~X;
    BLA::Matrix<2, 2> inversed_X_transpose_dot_X = X_transpose * X;
    BLA::Invert(inversed_X_transpose_dot_X);

    BLA::Matrix<2, 1> x_hat = inversed_X_transpose_dot_X * X_transpose * b;
    
    // intersection between wall-equation and perpendicular line.
    float x = (-x_hat(1, 0) * x_hat(0, 0) / (x_hat(1, 0) * x_hat(1, 0) + 1));
    float y = x_hat(0, 0) / (x_hat(1, 0) * x_hat(1, 0) + 1);

    Serial.print("b: ");
    Serial.print(x_hat(0, 0));
    Serial.print(" a: ");
    Serial.println(x_hat(1, 0));

    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.println(y);

    rotateToTarget(atan2(y, x) * 180 / M_PI);
}

int32_t getGyroAngle(void) {
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
    int currentAngle = getGyroAngle(); 

    while(-target != currentAngle && target != currentAngle) {
        currentAngle = getGyroAngle();

        if(target != currentAngle && target < (currentAngle)) motors.setSpeeds(turnSpeed, -turnSpeed);
        else if(target != currentAngle && target > (currentAngle)) motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    motors.setSpeeds(0,0);
}
