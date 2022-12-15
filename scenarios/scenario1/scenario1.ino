#include<Wire.h>
#include<Zumo32U4.h>
#include<BasicLinearAlgebra.h>
#include<math.h>

Zumo32U4IMU imu;
Zumo32U4Motors motors;

#include"TurnSensor.h"

const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;

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
    delay(1000);
    calibrateWall(trigPinF, echoPinF);
    Serial.println("current angle: " + (String)getGyroAngle()); 
}

void loop(void) {
    //Serial.println(getGyroAngle());
    //delay(1000);
}

void calibrateWall(const int trig, const int echo) {
    const int dataPointsTotal = 5;
    int angle;
    double length;
    int arr[dataPointsTotal] = {0, 10, 13, -10, -13};// {0, 5, 10, -5, -10};

    BLA::Matrix<dataPointsTotal, 2> X;
    BLA::Matrix<dataPointsTotal, 1> b;
      
    for(int i=0; i<dataPointsTotal; i++) {
        rotateToTarget(arr[i]);
        angle = getGyroAngle();
        length = getUltraSonicSensorData(trig, echo);
    
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
    Serial.println("current angle: " + (String) currentAngle);

    while(target != currentAngle) {
        currentAngle = getGyroAngle();
        if(target < currentAngle) motors.setSpeeds(turnSpeed, -turnSpeed);
        else if(target > (currentAngle)) motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    motors.setSpeeds(0,0);
}
