#include <AccelStepper.h>

const int dirPin = 10;      // set direction pin of left motor
const int stepPin = 9;      // set step pin of left motor 

const int dirPin2 = 8;      // set direction pin of right motor 
const int stepPin2 = 11;    // set step pin of right motor 

float conversion, curpos;

// creating both instances of the accelstepper class for left and right motor
AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);        
AccelStepper myStepper2(AccelStepper::DRIVER, stepPin2, dirPin2);


void setup() {
  // put your setup code here, to run once:

  conversion = 34.5575/1600;
  Serial.begin(31250);

  myStepper.setMaxSpeed(2000);        // set max speed of left motor 
  myStepper.setAcceleration(500);      // set acceleration in steps per sec^2
  myStepper.moveTo(1200);            // set position for motor to achieve in steps

  myStepper2.setMaxSpeed(2000);       // set max speed of right motor
  myStepper2.setAcceleration(500);      // set acceleration in microstep per sec^2 
  myStepper2.moveTo(-1200);           // set position for motor to achieve in microsteps 

}

void loop() {
  // put your main code here, to run repeatedly:

  //curpos = (myStepper.currentPosition())*conversion;

 // Serial.print("Current Position: ");
  //Serial.print(curpos);
  //Serial.println(" mm");

  if(!myStepper.run()){
    myStepper.moveTo(-myStepper.currentPosition());
  }
  if(!myStepper2.run()){
    myStepper2.moveTo(-myStepper2.currentPosition());
  }

}
