#include<Zumo32U4.h>

Zumo32U4OLED lcd;
Zumo32U4ProximitySensors dis;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

void setup() {
    // put your setup code here, to run once:
    dis.initThreeSensors();
    motors.setRightSpeed(100);
    motors.setLeftSpeed(100);
}

void loop() {
    // put your main code here, to run repeatedly:
    dis.read();
    int leftSensor = dis.countsLeftWithLeftLeds();
    int cLeftSensor = dis.countsFrontWithLeftLeds();
    int cRightSensor = dis.countsFrontWithRightLeds();
    int rightSensor = dis.countsRightWithRightLeds();

    lcd.gotoXY(0,1);
    lcd.print(leftSensor);
    lcd.print(" ");
    lcd.print(cLeftSensor);
    lcd.print(" ");
    lcd.print(cRightSensor);
    lcd.print(" ");
    lcd.print(rightSensor);
    lcd.print(" ");
    delay(10);

    if (rightSensor > 5) {
        motors.setRightSpeed(125);
        if (cRightSensor > 5) {
            motors.setSpeeds(0,0);
            delay(10);
            motors.setSpeeds(-100,-100);
            delay(1000);
            motors.setSpeeds(-100,0);
            delay(500);
            motors.setSpeeds(100,100);
        }
    }

    if (rightSensor < 5) {
        motors.setRightSpeed(75);
    }

    else if (rightSensor == 5) {
        motors.setRightSpeed(100);
    }
}
