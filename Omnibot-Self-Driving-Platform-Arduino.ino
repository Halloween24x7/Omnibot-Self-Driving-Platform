// Omnibot or Equivelant platform for obstacle avoidance using an arduino, motor controller and lidar sensors

/* Variables that may be changed based on your Bot's exact configuration */
// Set Motor Driver Speed.
int ABSMax = 235; // Left Wheels - 255 is Max Speed (Separate left and right allows for trimming to keep bot running in a straight line)
int BBSMax = 255; // Right Wheels

// Distances to turn or stop at. This will need to be changed depending on sensor placement.
int MinCenterDistance = 300; // Minimum Distance before taking action (mm)
int MinLRDistance = 300; // Minimun Distance from left and right sensors before taking action.
int StopDistance = 200; // Distance from object to stop (mm)

/* Less likely to Adjust */
// Speed Control Multiplier
float FifthGear = 1;
float FourthGear = 0.90;
float ThirdGear = 0.80;
float SecondGear = 0.50;
float FirstGear = 0.30;

// Gear Change Distances (mm) any of the front sensors triggers a gear shift
float FifthDistance = 1000;
float FourthDistance = 800;
float ThirdDistance = 500;
float SecondDistance = 300;
float FirstDistance = 200;
/* End Variables that may be changed based on your Bot's exact configuration */

#include <Wire.h>
#include <VL53L0X.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define NUM_SENSORS 3

// Motor Control
int EnA = 9; // Must be a PWM pin for speed control
int in1 = 10;
int in2 = 11;

int EnB = 5; // Must be a PWM pin for speed control
int in3 = 6;
int in4 = 7;

int ABS = 0; // Left Wheels Running Speed
int BBS = 0; // Right Wheels Running Speed

// Sensors
int RightDistance = 0;
int LeftDistance = 0;
int CenterDistance = 0;

void MoveTurnAround(int ABS, int BBS)
{
  analogWrite(EnA,ABS); analogWrite(EnB,BBS);
  digitalWrite(in1,HIGH); digitalWrite(in2,LOW);
  digitalWrite(in3,LOW); digitalWrite(in4,HIGH);
  Serial.println("Turn Around!");
  delay(100);
}

void MoveBack(int ABS, int BBS)
{
  analogWrite(EnA,ABS); analogWrite(EnB,BBS);
  digitalWrite(in1,HIGH); digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH); digitalWrite(in2,LOW);
  Serial.println("Reverse!");
}

void MoveForward(int ABS, int BBS)
{
  analogWrite(EnA,ABS); analogWrite(EnB,BBS);
  digitalWrite(in1,LOW); digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW); digitalWrite(in4,HIGH);
  Serial.println("Forward!");
}

void MoveLeft(int ABS, int BBS)
{
  analogWrite(EnA,ABS); analogWrite(EnB,BBS);
  digitalWrite(in1,HIGH); digitalWrite(in2,LOW);
  digitalWrite(in3,LOW); digitalWrite(in4,HIGH);
  Serial.println("Turn Left!");
}

void MoveRight(int ABS, int BBS)
{
  analogWrite(EnA,ABS); analogWrite(EnB,BBS);
  digitalWrite(in1,LOW); digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH); digitalWrite(in4,LOW);
  Serial.println("Turn Right!");
} 

  void MoveStop(int ABS, int BBS)
{
  analogWrite(EnA,0); analogWrite(EnB,0);
//  digitalWrite(in1,LOW); digitalWrite(in2,LOW);
//  digitalWrite(in3,LOW); digitalWrite(in4,LOW);
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

  pinMode(EnA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pinMode(EnB,OUTPUT);
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
  Serial.println("Let's Go...");
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i);

    VL53L0XPtr sensor = sensors[i];

//    Serial.print(i);
//    Serial.print("(");
//    Serial.print(sensor->last_status);
//    Serial.print(") => ");

    if (sensor->last_status == 0)
    {
//      Serial.print(sensor->readRangeContinuousMillimeters());
//      Serial.print(" mm");

      if(i==0)
      {
        LeftDistance = sensor->readRangeContinuousMillimeters();
      }
      if(i==1)
      {
        CenterDistance = sensor->readRangeContinuousMillimeters();
      }
      if(i==2)
      {
        RightDistance = sensor->readRangeContinuousMillimeters();
      }
    }

    if (sensor->timeoutOccurred())
    {
//      Serial.print(" [timeout]");
    }
//    Serial.print(";\t");  
  }
//  Serial.println();

  Serial.print("Left Distance: "); Serial.println(LeftDistance);
  Serial.print("Center Distance: "); Serial.println(CenterDistance);
  Serial.print("Right Distance: "); Serial.println(RightDistance);


  if ((LeftDistance < FirstDistance) || (CenterDistance < FirstDistance) || (RightDistance < FirstDistance)) { ABS = ABSMax*FirstGear; BBS = BBSMax*FirstGear; Serial.println("First Gear"); }
  else if ((LeftDistance < SecondDistance) || (CenterDistance < SecondDistance) || (RightDistance < SecondDistance)) { ABS = ABSMax*SecondGear; BBS = BBSMax*SecondGear; Serial.println("Second Gear"); }
  else if ((LeftDistance < ThirdDistance) || (CenterDistance < ThirdDistance) || (RightDistance < ThirdDistance)) { ABS = ABSMax*ThirdGear; BBS = BBSMax*ThirdGear; Serial.println("Third Gear"); }
  else if ((LeftDistance < FourthDistance) || (CenterDistance < FourthDistance) || (RightDistance < FourthDistance)) { ABS = ABSMax*FourthGear; BBS = BBSMax*FourthGear; Serial.println("Fourth Gear"); }
  else if ((LeftDistance >= FifthDistance) || (CenterDistance >= FifthDistance) || (LeftDistance >= FifthDistance)) { ABS = ABSMax*FifthGear; BBS = BBSMax*FifthGear; Serial.println("Fifth Gear"); }

// Drive Logic
  {
      if ((RightDistance >= MinLRDistance) && (CenterDistance >= MinCenterDistance) && (LeftDistance >= MinLRDistance))
      {
        MoveForward(ABS, BBS);
      }
      else if ((RightDistance < LeftDistance) || (RightDistance < MinLRDistance))
      {
        MoveLeft(ABS, BBS);
      }
      else if ((LeftDistance < RightDistance) || (LeftDistance < MinLRDistance))
      {
        MoveRight(ABS, BBS);
      }
    else
    {
      MoveStop(ABS, BBS);
    }  

  }
  delay(100);
}
