/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ahmad                                                     */
/*    Created:      Fri May 13 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// EastMotor            motor         1               
// NorthMotor           motor         2               
// SouthMotor           motor         10              
// WestMotor            motor         9               
// GPS16                gps           16              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

//test

#include "vex.h"
#include "drivetrain.h"
#include "mathlib.h"

bool buttonAState = 0;

using namespace vex;
competition Competition;
Drivetrain train(3.25, 1, NorthMotor, SouthMotor, EastMotor, WestMotor, 0, 0, 0, 0, GPS16);

void pre_auton(void){
  vexcodeInit();
  train = Drivetrain(3.25, 1, NorthMotor, SouthMotor, EastMotor, WestMotor, 0, 0, 0, 0, GPS16);


}

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
}

float turnTowardsPoint(int x, int y){
  float motorPercentage = 0;
  float Kp = 1;
  int error = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;
  
  desiredAngle = atan2(y - GPS16.yPosition(inches), x - GPS16.xPosition(inches)) * (180/M_PI);

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(GPS16.heading())) + 360) % 360);
  //finds the difference in the current angle and the desired angle from 0 to 360 degrees

  if (angleFromDesired > 180){
    error = -(180 - (angleFromDesired - 180));
  } else {
    error = angleFromDesired;
  }
  //translates angleFromDesired to -180 to 180 degrees

  motorPercentage = Kp * error;
  //calculates the desired voltage of the motors

  //printToController(desiredAngle, 1, 1);
    
  return(motorPercentage);
}

void testGoToMethod(){
  int randomAngle = 0;
  int xRandom = 0;
  int yRandom = 0; 

  srand((unsigned) Brain.Timer.time(msec));
  randomAngle = (rand() % 360) + 1;
  train.faceHeading(randomAngle);
  //This is to prove that the goTo method works no matter the heading of the robot

  srand((unsigned) Brain.Timer.time(msec));
  xRandom = (rand() % 1601) - 800;

  wait(rand() % 3 + 1, seconds);

  srand((unsigned) Brain.Timer.time(msec));
  yRandom = (rand() % 1601) - 800;
  //generates a set of random coordinates

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(xRandom);
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(yRandom);
  //This prints the desired position, that was randomly generated, on the screen of the controller

  train.goToPos(xRandom, yRandom);
  //This sends the robot to a random x and y position

  Controller1.Screen.setCursor(1,7);
  Controller1.Screen.print(GPS16.xPosition(mm));
  Controller1.Screen.setCursor(2,7);
  Controller1.Screen.print(GPS16.yPosition(mm));
  //This prints where the robot actually went on the screen of the controller

  return;
}

void autonomous(void){
  // train.goToPos(0, 0);

  testGoToMethod();
  


}

void usercontrol(void){
  int storedPercentage = 0;

  while(true){


    if (buttonAState == 1){
      storedPercentage = turnTowardsPoint(0, 0);
    } else {
      storedPercentage = 0;
    }
    //This will allow the robot to turn to face a goal while still being able to be driven around
    train.steeringControl(Controller1, storedPercentage);

    //This allows for driver control. By modifying the value outputed by the control stick, the movement of the robot is relative to the field, rather than the heading.

    

  }
}



int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
