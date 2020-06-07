// Omnibot or Equivelant platform for obstacle avoidance using an arduino, motor controller and lidar sensors
// Some code comes from KB3HXA obstacle avoidance code which uses sonic sensors.

#include <Wire.h>
#include <VL53L0X.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define NUM_SENSORS 3

// Set up motor driver I/O

int ENA = 9; // Must be a PWM pin for speed control
int in1 = 10;
int in2 = 11;

int ENB = 5; // Must be a PWM pin for speed control
int in3 = 6;
int in4 = 7;

// Set motor driver speed.
// Added BBS because one of my motors ran slightly faster than the other. This allows trimming so the bot can drive in a straight line.
// It would be better to trim using an analog input, however, we've used all of them on the UNO.

int ABSMax = 235; // Left Wheels
int BBSMax = 255; // Right Wheels

int ABS = 90; // Left Wheels
int BBS = 105; // Right Wheels

// Distances to turn or stop at. This will need to be changed depending on sensor placement.
int MinDistance = 300; // Minimum Distance before taking action (mm)
int StopDistance = 200; // Distance from object to stop (mm)

// Sensors
int RightDistance = 0;
int LeftDistance = 0;
int CenterDistance = 0;

// Speed Control
float FifthGear = 1;
float FourthGear = 0.80;
float ThirdGear = 0.60;
float SecondGear = 0.40;
float FirstGear = 0.30;

float FifthDistance = 800;
float FourthDistance = 700;
float ThirdDistance = 600;
float SecondDistance = 500;
float FirstDistance = 400;

void MoveBack()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,BBS);
 digitalWrite(in1,HIGH);
 digitalWrite(in2,LOW);
  
 digitalWrite(in3,HIGH);
 digitalWrite(in2,LOW);
 
 Serial.println("Reverse!");
}

void MoveForward()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,BBS);
 digitalWrite(in1,LOW);
 digitalWrite(in2,HIGH);
 digitalWrite(in3,LOW);
 digitalWrite(in4,HIGH);

 Serial.println("Forward!");
}

void MoveLeft()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,BBS);
 digitalWrite(in1,HIGH);
 digitalWrite(in2,LOW);
 digitalWrite(in3,LOW);
 digitalWrite(in4,HIGH);
 Serial.println("Turn Left!");
}

void MoveRight()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,BBS);
 digitalWrite(in1,LOW);
 digitalWrite(in2,HIGH);
 digitalWrite(in3,HIGH);
 digitalWrite(in4,LOW);
 Serial.println("Turn Right!");
} 

  void MoveStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  Serial.println("Stop!");
} 


// Check LIDAR Sensors
void tcaselect(uint8_t i) {
  if (i > 3) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  int result = Wire.endTransmission();
}

typedef VL53L0X* VL53L0XPtr;

VL53L0XPtr sensors[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("starting...");

  pinMode(ENA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pinMode(ENB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i);

    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();
    sensor->setTimeout(100);
    sensor->setMeasurementTimingBudget(33000);
    sensor->startContinuous();
    sensors[i] = sensor;
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i);

    VL53L0XPtr sensor = sensors[i];

    Serial.print(i);
    Serial.print("(");
    Serial.print(sensor->last_status);
    Serial.print(") => ");

    if (sensor->last_status == 0)
    {
      Serial.print(sensor->readRangeContinuousMillimeters());
      Serial.print(" mm");

      if(i==0)
      {
        LeftDistance = sensor->readRangeContinuousMillimeters();
      }
            if(i==1)
      {
        CenterDistance = sensor->readRangeContinuousMillimeters();

        if (CenterDistance >= FifthDistance) { ABS = ABSMax*FifthGear; BBS = BBSMax*FifthGear; Serial.println("Fifth Gear"); }
        else if (CenterDistance >= FourthDistance) { ABS = ABSMax*FourthGear; BBS = BBSMax*FourthGear; Serial.println("Fourth Gear"); }
        else if (CenterDistance >= ThirdDistance) { ABS = ABSMax*ThirdGear; BBS = BBSMax*ThirdGear; Serial.println("Third Gear"); }
        else if (CenterDistance >= SecondDistance) { ABS = ABSMax*SecondGear; BBS = BBSMax*SecondGear; Serial.println("Second Gear"); }
        else if (CenterDistance >= FirstDistance) { ABS = ABSMax*FirstGear; BBS = BBSMax*FirstGear; Serial.println("First Gear"); }
      }
            if(i==2)
      {
        RightDistance = sensor->readRangeContinuousMillimeters();
      }
      
    }

    if (sensor->timeoutOccurred()) {
      Serial.print(" [timeout]");
    }
    Serial.print(";\t");
  }

  Serial.println();

  delay(100);





  // Program instructions.

    if ((RightDistance >= MinDistance) && (CenterDistance >= MinDistance) && (LeftDistance >= MinDistance)) // Move Forward
    {
      MoveForward();
    }
    else if ((RightDistance < LeftDistance) || (RightDistance < MinDistance))
    {
      MoveLeft();
    }
    else if ((LeftDistance < RightDistance) || (LeftDistance < MinDistance))
    {
      MoveRight();
    }
    else
    {
      MoveStop();
    }

    if (CenterDistance <= StopDistance) 
    {
      MoveStop();
    }
}
