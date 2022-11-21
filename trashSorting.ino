#include <Wire.h>
#include <Zumo32U4.h>

// For this code to work, the library proximiditysensors must be edited, 
// such the prepareRead() func does not turn emittersOff.

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;

unsigned int lineSensorValues[5];
int brightnessLevels[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

void calibrateSensors() {
  delay(1000);
  // Rotates in place to sweep sensors over line
  for (uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) motors.setSpeeds(-100, 100);
    else motors.setSpeeds(100, -100);
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
  float kp = 0.2, kd = 1;
  int sensorInput, error, lastError = 0, errorRate, regulatedSpeed;
  
  while (lineSensorValues[0] < 300 && lineSensorValues[4] < 300 && lineSensorValues[2] > 200) {
    sensorInput = lineSensors.readLine(lineSensorValues);
    error = sensorInput - setpoint;
    errorRate = error - lastError;
    regulatedSpeed = baseSpeed + (error*kp + errorRate*kd);
    motors.setSpeeds(baseSpeed, regulatedSpeed);
    lastError = error;
  }

  motors.setSpeeds(0, 0);
}

void scanTrash() {
  proxSensors.initFrontSensor();
  proxSensors.setBrightnessLevels(brightnessLevels, 20);
  while(true){
  proxSensors.read();
  delay(100);
  display.clear();
  int cLeftSensor = proxSensors.countsFrontWithLeftLeds();
  int cRightSensor = proxSensors.countsFrontWithRightLeds();
  lineSensors.emittersOn();
  display.println(cLeftSensor);
  display.println(cRightSensor);
  if(cLeftSensor == 5 || cRightSensor == 5){
    display.clear();
    display.println("Close");
    }
  //else if (cLeftSensor == 5 || cRightSensor == 5){
   // display.clear();
    //display.println("Far");
    
  //}
}
}

void moveOntoLine() {
  // Move a bit past line
  motors.setSpeeds(100, 100);
  delay(600);

  // Turn Anticlockwise until the right sensor hits the line.
  motors.setSpeeds(100, -100);
  while(lineSensorValues[2] >= 150) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0);
      
}

void findLine() {
  motors.setSpeeds(100, 100);
  while (!(lineSensorValues[0] < 400 || lineSensorValues[4] < 400 || lineSensorValues[2] < 400)) {
    lineSensors.readCalibrated(lineSensorValues);
  }
  motors.setSpeeds(0, 0);
}

void setup(){
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

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());

  findLine();
  moveOntoLine();
  followLine();
  scanTrash();
}

void loop(){ 

}
