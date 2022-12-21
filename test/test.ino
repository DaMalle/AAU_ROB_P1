#include<Wire.h>
//#include<Zumo32U4.h>
#include <VL53L0X.h>

VL53L0X tof;

const int trigPinF = 1, trigPinB = 30, trigPinL = 17, trigPinR = 0,
          echoPinF = 4, echoPinB = 20, echoPinL = 18, echoPinR = 21;
 
void setup() {
  Serial.begin(9600);
  Wire.begin();
  I2Cscan();

  /*pinMode(trigPinR, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  //--------------------------
  pinMode(echoPinR, INPUT);
  pinMode(echoPinB, INPUT);
  pinMode(echoPinF, INPUT);
  pinMode(echoPinL, INPUT);*/
  
  tof.init();
  tof.setTimeout(500);
  tof.setMeasurementTimingBudget(100000);

  //tof.startContinuous();
}    
 
void loop() {
  Serial.println(tof.readRangeSingleMillimeters());
  //I2Cscan();
  //delay(1000);
}

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

double getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); // clear
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // stop pulse

  // Return distance
  return pulseIn(echo, HIGH) * 0.034029 / 2; // Speed of sound wave divided by 2 (go and back)
}