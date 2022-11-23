#include <Wire.h>
#include <Zumo32U4.h>


// For this code to work, the library proximitysensors must be edited, 
// such the prepareRead() func does not turn emittersOff.

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;

#include "TurnSensor.h"

unsigned int lineSensorValues[5];
int brightnessLevels[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
char TrashDistance;
int currentAngle = 0;


void rotateToTarget(int target) {  // span {-180 -- 0 -- 180}
  while(true) {
    turnSensorUpdate();
    int currentAngle = getAngle();
    if(target != currentAngle && target < (currentAngle)) { 
      motors.setSpeeds(100, -100);
      display.clear();
      display.print((String)currentAngle);
    }
      else if(target != currentAngle && target > (currentAngle)) {
      motors.setSpeeds(-100, 100);
      display.clear();
      display.print((String)currentAngle);
    }
    else if(-target == currentAngle || target == currentAngle) {
      motors.setSpeeds(0,0);
      break;
    }
  }
}

int getAngle() {
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

void calibrateSensors() {
  delay(1000);
  // Rotates in place to sweep sensors over line
  for(uint16_t i = 0; i < 120; i++) {
    if(i > 30 && i <= 90) motors.setSpeeds(-150, 150);
    else motors.setSpeeds(150, -150);
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void showReadings() {
  /* Prints live measurements until button A is pressed. */
  display.clear();
  while(!buttonA.getSingleDebouncedPress()) {
    lineSensors.readCalibrated(lineSensorValues);
    display.gotoXY(0, 0);
    display.println(lineSensorValues[0]);//Left
    display.gotoXY(4, 0);
    display.println(lineSensorValues[4]);//Right
    display.gotoXY(0, 1);
    display.println(lineSensorValues[2]);//Middle
  }
}

void showLineSensorValues() {
  /* Reads linesensor values. Values span from 0-1000. Higher value = more color contrast. */
  lineSensors.readCalibrated(lineSensorValues);
  display.clear();
  display.println(lineSensorValues[1]);
  display.gotoXY(4,0);
  display.println(lineSensorValues[3]);
  display.gotoXY(0,1);
  display.println(lineSensorValues[2]);
}

void followLine() {
  const int setpoint = 2000, baseSpeed = 100;
  float kp = 0.8, kd = 1;
  int sensorInput, error, lastError = 0, errorRate, regulatedSpeed;
  
  lineSensors.readCalibrated(lineSensorValues);
  while(!(lineSensorValues[0] < 300 && lineSensorValues[4] < 300 && lineSensorValues[2] > 200)) {
    sensorInput = lineSensors.readLine(lineSensorValues);
    error = sensorInput - setpoint;
    errorRate = error - lastError;
    regulatedSpeed = baseSpeed + (error*kp + errorRate*kd);
    motors.setSpeeds(baseSpeed, regulatedSpeed);
    lastError = error;
  }

  motors.setSpeeds(0, 0);
}

char scanTrash() {
  int cLeftSensor;
  int cRightSensor;

  display.clear();
  lineSensors.emittersOn();

  while(true) {
    proxSensors.read();
    cLeftSensor = proxSensors.countsFrontWithLeftLeds();
    cRightSensor = proxSensors.countsFrontWithRightLeds();
    if(cLeftSensor == cRightSensor && cRightSensor == 20) {
      display.gotoXY(0, 1);
      display.println("Close");
      delay(600);
      lineSensors.emittersOff();
      return 'c';
    }
    if((cRightSensor == cLeftSensor && cRightSensor > 17 && cRightSensor < 20)) {
      display.gotoXY(0, 1);
      display.println("Far");
      lineSensors.emittersOff();
      return 'f';
    }
  }
}

void moveOntoLine() {
  // Move a bit past line
  motors.setSpeeds(100, 100);
  delay(550);

  // Turn Anticlockwise until the right sensor hits the line.
  motors.setSpeeds(100, -100);
  while(lineSensorValues[2] >= 150) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0);
      
}

void findLine() {
  motors.setSpeeds(300, 300);
  lineSensors.readCalibrated(lineSensorValues);
  while(!detectLine(100)) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0);
}

bool detectLine(const int lineValue) {
  return lineSensorValues[0] < lineValue || lineSensorValues[4] < lineValue || lineSensorValues[2] < lineValue;
}

void sortTrash() {
  switch(TrashDistance) {
    case 'f': sortTrashFar(); break;
    case 'c': sortTrashClose(); break;
    default: break;
  }
}

void sortTrashFar(){

  // move away from start line
  motors.setSpeeds(300, 300);
  delay(200);

  // move forward until a new line is found
  findLine();
  buzzer.play("L16 cdegreg4");
  TrashDistance='a';

  // return to start line
  motors.setSpeeds(-300, -300);
  delay(300);
  lineSensors.readCalibrated(lineSensorValues);
  while(!detectLine(100)) lineSensors.readCalibrated(lineSensorValues);
  delay(200);
  followLine();
}

void sortTrashClose() {

  // go around trash and push
  rotateToTarget(-30);
  motors.setSpeeds(300, 300);
  delay(400);
  motors.setSpeeds(0, 0);
  rotateToTarget(90);  
  lineSensors.readCalibrated(lineSensorValues);
  findLine();
  buzzer.play("L16 cdegreg4");

  motors.setSpeeds(100, 100);
  delay(120);
  motors.setSpeeds(0, 0);
  turnSensorReset();
  rotateToTarget(90);
  motors.setSpeeds(200, 200);
  delay(900);
  motors.setSpeeds(0, 0);
  rotateToTarget(169);
  findLine();

  motors.setSpeeds(100, 100);
  delay(550);

  // Turn clockwise until middle sensor hits the line.
  motors.setSpeeds(-100, 100);
  while(lineSensorValues[2] >= 150) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0);

  followLine();
  TrashDistance='a';
}

void setup() {
  //turnSensorSetup();  
  proxSensors.initFrontSensor();
  proxSensors.setBrightnessLevels(brightnessLevels, 20);
  lineSensors.initFiveSensors();
  display.clear();
  
   
  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Wait for button A to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonA.waitForButton();

  calibrateSensors();
  showReadings();
  buttonA.waitForButton();

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());

  findLine();
  moveOntoLine();
  followLine();
  turnSensorSetup(); 
}

void loop() {
  turnSensorReset(); 
  TrashDistance = scanTrash();
  sortTrash();
}