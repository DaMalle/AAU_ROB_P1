
#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
const uint16_t maxSpeed = 100;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonA buttonB;
Zumo32U4OLED display;

int16_t lastError = 0;
#define NUM_SENSORS 5   //amount of sensors.
unsigned int lineSensorValues[NUM_SENSORS];

void calibrateSensors(){
  display.clear();
  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-100, 100);
    }
    else
    {
      motors.setSpeeds(100, -100);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Shows sensor readings on the display.
void showReadings(){
  display.clear();
  while(!buttonA.getSingleDebouncedPress())//This function will print the live measurements until the button A is pressed.
  {
    lineSensors.readCalibrated(lineSensorValues);
    display.gotoXY(0, 0);
    display.println(lineSensorValues[0]);//Left
    display.gotoXY(4, 0);
    display.println(lineSensorValues[4]);//Right
    display.gotoXY(0, 1);
    display.println(lineSensorValues[2]);//Middle
  }
}
void followLine(){

}

void landOnLine(){
  //Sensor 0 = Left
  //Sensor 4 = Right
  while (true){
  lineSensors.readCalibrated(lineSensorValues);// denne funktion bliver kaldt efter at robotten har fundet linjen.
  //                                the robot will turn Anticlockwise until the right sensor hits the line.
    motors.setSpeeds(0,80);
    //                              printing rightsensor measurements.
    display.gotoXY(0,1);
    display.println(lineSensorValues[4]);
    //                              After the rightsensor see's the line. This section checks if both the right and the left sensor is on the line. then the robot will turn around and land on the line.
    if(lineSensorValues[4]>800){
      motors.setSpeeds(0,0);
      if(lineSensorValues[4]>800 && lineSensorValues[0]>800){
      delay(10);
      motors.setSpeeds(100,100);
      delay(460);
      motors.setSpeeds(100,-100);
      delay(1080);
      motors.setSpeeds(0,0);
      break;
      }
      else if(lineSensorValues[4]<800){// if both sensors are not on the line this section will try to. 
        motors.setSpeeds(0,-180);
        }
      else if(lineSensorValues[0]<800){
        motors.setSpeeds(-180,0);
      }
      }
}}

bool findLine(){
  while (true){
  lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(100,100);
  if(lineSensorValues[0] > 400 ||lineSensorValues[4] > 400 ||lineSensorValues[2] > 400 ){
    motors.setSpeeds(0,0);
    return true;
  }
  }
}

void setup(){
  lineSensors.initFiveSensors();
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
  landOnLine();
}

void loop(){






}
