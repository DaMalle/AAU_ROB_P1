
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonA buttonB;
Zumo32U4OLED display;
Zumo32U4ProximitySensors dis;
#define NUM_SENSORS 5   //amount of sensors.
unsigned int lineSensorValues[NUM_SENSORS];
int p = 2;
int D = 1;
int lastError = 0;
int errorRate = 0;

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
int regulatedSpeed = 100;
void followLine(){
  
  while(true){
    //1 = Cleft
    //3 = CRight
    //2 = middle
  if(lineSensorValues[1] < 300 && lineSensorValues[3] < 300 && lineSensorValues[2] > 200 ){
   
    motors.setSpeeds(0, 0);
    break;
  }
  else{
  lineSensors.readCalibrated(lineSensorValues); //this function reads the linesensor values. the values span from 0-1000, the higher the value, the more color contrast. 
  motors.setSpeeds(100,regulatedSpeed);
  display.clear();
  display.println(lineSensorValues[1]);
  display.gotoXY(4,0);
  display.println(lineSensorValues[3]);
  display.gotoXY(0,1);
  display.println(lineSensorValues[2]);
  int pos = lineSensors.readLine(lineSensorValues);
  if(pos<2000){//left
    int Error = 0;
    Error = (pos-2000)/10;
    errorRate = Error-lastError;
    regulatedSpeed = 100 + (Error*p+errorRate*D);
    lastError = Error;
  }
  else if(pos>2000){//right
  int Error = 0;
    Error = (2000-pos)/10;
    errorRate = Error-lastError;
    regulatedSpeed = 100 - (Error*p+errorRate*D);
    lastError = Error;
    }
  
  else if(pos == 2000){
    regulatedSpeed = 100;
  }
  
  }
}}

void scanCan(){
  dis.initFrontSensor();
  while(true){
  dis.read();
  display.clear();
  int cLeftSensor = dis.countsFrontWithLeftLeds();
  int cRightSensor = dis.countsFrontWithRightLeds();
  lineSensors.emittersOn();
  display.println(cLeftSensor);
  if(cLeftSensor == 6 || cRightSensor == 6){
    display.clear();
    display.println("Close");
    break;
  }
  else if (cLeftSensor == 5 || cRightSensor == 5){
    display.clear();
    display.println("Far");
    break;
  }
}}

void landOnLine(){// denne funktion bliver kaldt efter at robotten har fundet linjen.
  //Sensor 0 = Left
  //Sensor 4 = Right
  while (true){
  lineSensors.readCalibrated(lineSensorValues);
  //                                the robot will turn Anticlockwise until the right sensor hits the line.
    motors.setSpeeds(0,100);
    //                              printing rightsensor measurements.
    display.gotoXY(0,1);
    display.println(lineSensorValues[4]);
    //                              After the rightsensor see's the line. This section checks if both the right and the left sensor is on the line. then the robot will turn around and land on the line.
    if(lineSensorValues[4]<300){
      motors.setSpeeds(0,0);
      if(lineSensorValues[4]<300 && lineSensorValues[0]<300){
      delay(10);
      motors.setSpeeds(100,100);
      delay(650);
      motors.setSpeeds(0,0);
      break;
      }
      else if(lineSensorValues[4]>300 && lineSensorValues[0] < 300 ){// if both sensors are not on the line this section will try to. 
        motors.setSpeeds(0,-180);
        }
      else if(lineSensorValues[0]>300 && lineSensorValues[4] < 300){
        motors.setSpeeds(-180,0);
      }
      else{
        motors.setSpeeds(-100, -100);
      }
      }
}}

void turn(){
  while(true){
  lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(100, -100);
  if (lineSensorValues[2]<150){
    motors.setSpeeds(0,0);
    break;
  }
  }
}

void findLine(){
  //0 = left
  //4 = Right
  //2 = middle
  while (true){
  lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(100,100);
  if(lineSensorValues[0] < 400 ||lineSensorValues[4] < 400 ||lineSensorValues[2] < 400 ){
    motors.setSpeeds(0,0);
    break;
  }
  }
}

void setup(){
  Serial.begin(9600);
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
  turn();
  followLine();
  scanCan();

}

void loop(){ 

}
