#include<Zumo32U4.h>
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;                          //Man skal self importere det man skal bruge.
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;

const int echoPinR 0, echoPinL 2, echoPinF 14, echoPinB 3,//Disse pins er til ultralydssensorene
          trigPinR 1, trigPinL 18, trigPinF 13, trigPinB 21;

double getDistance(const int trig, const int echo) {//  Denne funktion kan bruges til at læse data fra en sensor.
    digitalWrite(trig, LOW); // clear
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); // Trigger pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW); // stop pulse
    return pulseIn(echo, HIGH) / 29 / 2; // Speed of sound wave divided by 2 (go and back)
}
//for at bruge funktionen skal man kalde den i setup eller i loop. 

void setup(){
//man skal lave en variabel som man gemmer dataene i (dataene kommer ud i cm.)  

double data = getDistance(trigPinR, echoPinR);         //Sådan bruger man funktionen. her læser man højre sensor (R-Right)

}


//PID constants
double kp = 1; 
double ki = 0.00; 
double kd = 1;   



double computePID(double input){  // her er input.   
  //currentTime = millis();                //get current time
  //elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - input;          // P - regulering
  
  cumError += error;               // I - regulering
  
  rateError = (error - lastError);   // D - regulering
  
  double out = kp*error + ki*cumError + kd*rateError;                //PID output               

  lastError = error;                                //remember current error


  return out;                                        //have function return the PID output
}

//igen skal man gemme dataene i en variabel.

double outoutPID = computePID(double input)
