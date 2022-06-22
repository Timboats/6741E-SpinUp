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
bool buttonBState = 0;

using namespace vex;
competition Competition;
Drivetrain train(3.25, 1, NorthMotor, SouthMotor, EastMotor, WestMotor, 0, 0, 0, 0, GPS16);

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
  return;
}

void toggleButtonB(){
  if (buttonBState == 0){
    buttonBState = 1;
  } else {
    buttonBState = 0;
  }
  return;
}

void pre_auton(void){
  vexcodeInit();
  train = Drivetrain(3.25, 1, NorthMotor, SouthMotor, EastMotor, WestMotor, 0, 0, 0, 0, GPS16);

  Controller1.ButtonA.pressed(toggleButtonA);
  Controller1.ButtonB.pressed(toggleButtonB);
}

int errorTTP = 0;
int prevErrorTTP = 0;
//There is no loop inside the turnTowardsPoint function
//So, these variables needed to be global
//They have TTP at the end to indicate this (Turn Towards Point)

float turnTowardsPoint(int x, int y){
  float motorPercentage = 0;
  const float Kp = 1;
  const float Ki = 0.01;
  const float Kd = 0.3;
  const int windupUpperLimit = 15;
  float integral = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;

  desiredAngle = atan2(y - GPS16.yPosition(inches), x - GPS16.xPosition(inches)) * (180/M_PI);

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(GPS16.heading())) + 360) % 360);
  //finds the difference in the current angle and the desired angle from 0 to 360 degrees

  if (angleFromDesired > 180){
    errorTTP = -(180 - (angleFromDesired - 180));
  } else {
    errorTTP = angleFromDesired;
  }
  //translates angleFromDesired to -180 to 180 degrees

  integral = integral + errorTTP;
  //Adds up the area of the error versus time graph from a specified time until the current time

  if (errorTTP > windupUpperLimit){
    integral = 0;
  }
  //Defines the lower limit of the integral

  motorPercentage = -(Kp * errorTTP) + (Ki*integral) + (Kd*(errorTTP - prevErrorTTP));
  //calculates the desired voltage of the motors

  prevErrorTTP = errorTTP; 
  //Used to calculate integral
    
  return(motorPercentage);
  //Value is fed into motor to facillitate desired action
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

float totalError = 0;
float averError = 0;
float prevTime = 0;
float deltaTime = 0;
float numOfInterations = 0;

float ttpErrorOverTime(){

  totalError = totalError + Simpler::abs(errorTTP);

  averError = totalError/numOfInterations;

  return(averError);
}

void autonomous(void){
  // train.goToPos(0, 0);

  testGoToMethod();
  


}

int ranX = 0;
int ranY = 0;

void usercontrol(void){

  int storedPercentage = 0;

  srand((unsigned) Brain.Timer.time(msec));
  wait((rand() % 201), msec);

  srand((unsigned) Brain.Timer.time(msec));
  ranX = (rand() % 141) - 70;

  wait(100, msec);

  srand((unsigned) Brain.Timer.time(msec));
  ranY = (rand() % 141) - 70;

  while(true){

    if (buttonBState == 1){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(ttpErrorOverTime());
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(totalError);
      Controller1.Screen.setCursor(3,1);
      Controller1.Screen.print(numOfInterations);
      Controller1.Screen.setCursor(2,10);
      Controller1.Screen.print(ranX);
      Controller1.Screen.setCursor(3,10);
      Controller1.Screen.print(ranY);
      numOfInterations = numOfInterations + 1;
    } else {
      numOfInterations = 0;
      totalError = 0;
    }

    

    if (buttonAState == 1){
      storedPercentage = turnTowardsPoint(ranX, ranY);
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
