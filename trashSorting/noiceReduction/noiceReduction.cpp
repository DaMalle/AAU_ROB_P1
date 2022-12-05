#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4Motors motors;

#define echoPinR 21
#define echoPinB 20
#define echoPinF 4
#define echoPinL 18
//--------------------
#define trigPinR 0
#define trigPinB 30
#define trigPinF 1
#define trigPinL 

//arrays for noice reduction. 
int arraySize = 10;
double array[arraySize];
 
void setup(){
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  //--------------------------
  pinMode(echoPinR,INPUT);
  pinMode(echoPinB,INPUT);
  pinMode(echoPinF,INPUT);
  pinMode(echoPinL,INPUT);        
  Serial.begin(9600);
  noiceReduction nR;
  nR.resetArrays();
}    
 
void loop(){
noiceReduction nR;
Serial.println(nR.getDistance());
}
 
 
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
        }
        else{ // if the readings are not within the errormargin.
        resetArrays();
        }
    }
  return sum/arraySize// Return average distance
  }

  void resetArrays(){
    for (int i = 0; i < arraySize; i++) //size of divided by 8 cuz its a double
    {
        array[i]=0;
    }
    
  }




  





};




