#include "I2Cdev.h"
#include "MPU6050.h"
#include <AccelStepper.h>

const int dirPin = 10;      // set direction pin of left motor
const int stepPin = 9;      // set step pin of left motor 

const int dirPin2 = 8;      // set direction pin of right motor 
const int stepPin2 = 11;    // set step pin of right motor 

float conversion, curpos;

// creating both instances of the accelstepper class for left and right motor
AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);        
AccelStepper myStepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

MPU6050 accelgyro;

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

double gyroX1,gx,gyroY1,gy,gyroZ1,gz,accelX1,ax,accelY1,ay,accelZ1,az;

unsigned long previousMillis = 0,startTime;  // Variable to store the last time something was printed
const long interval = 50;        // Interval in milliseconds (e.g., print every .1 second)

bool codeHasRun = false;  // Flag to keep track of whether the code has run

void setup() {
  //Serial.println("hello");

  // Initialize serial communication
  Serial.begin(31250);
  conversion = 34.5575/1600;    // conversion for microsteps to mm


  // Initialize the MPU6050 sensor
  Wire.begin();
  accelgyro.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Calibrate the sensor
  Serial.println("Calibrating sensor...");
  accelgyro.CalibrateAccel(6);
  accelgyro.CalibrateGyro(6);
  Serial.println("Sensor calibration complete.");


  Serial.println("hello");

  myStepper.setMaxSpeed(6000);        // set max speed of left motor 
  myStepper.setAcceleration(2000);      // set acceleration in steps per sec^2
  myStepper.moveTo(1200);            // set position for motor to achieve in steps

  myStepper2.setMaxSpeed(6000);       // set max speed of right motor
  myStepper2.setAcceleration(2000);      // set acceleration in microstep per sec^2 
  myStepper2.moveTo(-1200);           // set position for motor to achieve in microsteps 
 
  
  
  // Send header for CSV
  Serial.println("Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),ground truth pos (mm)");
}

void loop() {
  // put your main code here, to run repeatedly:

  curpos = (myStepper.currentPosition())*conversion;    //get current position of motors in mm

  // Check if it's time to run the first part of the loop
  if (millis() - startTime < 5000 || millis()-startTime > 25000) {  // Run for 5 seconds
    // Code for the first part of the loop
    // Read accelerometer and gyroscope data
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  accelgyro.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

//convert integer values into double values
  gyroX1 = gyroX;
  gyroY1 = gyroY;
  gyroZ1 = gyroZ;

  accelX1 = accelX;
  accelY1 = accelY;
  accelZ1 = accelZ;

//convert bits into units of m/s
  gx=gyroX1/131;
  gy=gyroY1/131;
  gz=gyroZ1/131;

  ax=(accelX1/16384)*9.81;
  ay=(accelY1/16384)*9.81;
  az=(accelZ1/16384)*9.81;

 unsigned long currentMillis = millis();  // Get the current time

  // Check if it's time to print something
  if (currentMillis - previousMillis >= interval) {
    // Save the current time as the last time something was printed
    previousMillis = currentMillis;

    // Print sensor readings
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.println(curpos);

  }
 
  } else {
    // Read accelerometer and gyroscope data
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  accelgyro.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

//convert integer values into double values
  gyroX1 = gyroX;
  gyroY1 = gyroY;
  gyroZ1 = gyroZ;

  accelX1 = accelX;
  accelY1 = accelY;
  accelZ1 = accelZ;

//convert bits into units of m/s
  gx=gyroX1/131;
  gy=gyroY1/131;
  gz=gyroZ1/131;

  ax=(accelX1/16384)*9.81;
  ay=(accelY1/16384)*9.81;
  az=(accelZ1/16384)*9.81;

 unsigned long currentMillis = millis();  // Get the current time

  // Check if it's time to print something
  if (currentMillis - previousMillis >= interval) {
    // Save the current time as the last time something was printed
    previousMillis = currentMillis;

    // Print sensor readings
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.println(curpos);

  }
  // Tell the steppers to run 
    if(!myStepper.run()){
    myStepper.moveTo(-myStepper.currentPosition());
  }
  if(!myStepper2.run()){
    myStepper2.moveTo(-myStepper2.currentPosition());
  }


    
  }

  
  // Save the start time of the loop if it hasn't been saved yet
  if (startTime == 0) {
    startTime = millis();
  }

}

