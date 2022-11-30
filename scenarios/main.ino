#include <Wire.h>
#include <Zumo32U4.h>

// For this code to work, the library proximitysensors must be edited, 
// such the prepareRead() func does not turn emittersOff.

Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;

#include "TurnSensor.h"

const int echoPinR 0, echoPinL 2, echoPinF 14, echoPinB 3,
          trigPinR 1, trigPinL 18, trigPinF 13, trigPinB 21;

int currentAngle = 0, baseSpeed = 100;


void setup() {
    Serial.begin(9600);

    pinMode(trigPinR, OUTPUT);
    pinMode(trigPinB, OUTPUT);
    pinMode(trigPinF, OUTPUT);
    pinMode(trigPinL, OUTPUT);

    pinMode(echoPinR, INPUT);
    pinMode(echoPinB, INPUT);
    pinMode(echoPinF, INPUT);
    pinMode(echoPinL, INPUT);
}

void loop() {
}

void regulateBaseSpeed() { // scenario 1
    const int safeguardedSpace = 25, protectiveStopSpace = 5;
    int left = getDistance(trigPinL, echoPinL);
    if (left <= protectiveStopSpace && )
    eles if left safeguardedSpace
}

double getDistance(const int trig, const int echo) {
    digitalWrite(trig, LOW); // clear
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); // Trigger pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW); // stop pulse

    // Return distance
    return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}
