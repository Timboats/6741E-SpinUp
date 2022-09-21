#include "main.h"
#include "drivetrain.h"
#include "mathlib.h"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <string>
#include "braingui.h"
#include "vexfs.h"
#include "inertialpos.h"

#define NORTHMOTORPORT 10
#define SOUTHMOTORPORT 11
#define EASTMOTORPORT 20
#define WESTMOTORPORT 1
#define GPS1PORT 15
#define LAUNCHERMOTORLEFTPORT 9
#define LAUNCHERMOTORRIGHTPORT 2
#define INERTIALSENSORPORT 8


bool buttonAState = 0;
int driveDirection = 1;

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
Drivetrain train(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 315, 135, GPS1PORT);

//TODO button callbacks still might not work problem-digital_new could be helpful tho

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
  return;
}

void initialize() {
  fileSysInit(); //Checks if all needed files exist. If not it creates them
  //TODO replace the below line with the custom init
  lvglInitEx();
  

  pros::Motor EastMotorInit(EASTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor NorthMotorInit(NORTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor SouthMotorInit(SOUTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor WestMotorInit(WESTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Gps GpsPrimaryInit(GPS1PORT, 0.00, 0.23);
  pros::Imu Inertial(INERTIALSENSORPORT);

  Inertial.reset();

  
  

  train = Drivetrain(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, GPS1PORT);


	//Controller1.ButtonA.pressed(toggleButtonA);

}

void controllerButtonCalls(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == true){
    driveDirection = -driveDirection;
  }

}

int errorTTP = 0;
int prevErrorTTP = 0;
//There is no loop inside the turnTowardsPoint function
//So, these variables needed to be global
//They have TTP at the end to indicate this (Turn Towards Point)

float turnTowardsPoint(int x, int y){
  pros::Gps GpsPrimary(GPS1PORT);
  float motorPercentage = 0;
  const float Kp = 1;
  const float Ki = 0.01;
  const float Kd = 0.3;
  const int windupUpperLimit = 15;
  float integral = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;

  desiredAngle = atan2(y - (GpsPrimary.get_status().y * 1000), x - (GpsPrimary.get_status().x * 1000)) * (180/M_PI); //in mm prolly gonna break

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(GpsPrimary.get_heading())) + 360) % 360);
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
  //Value is fed into motor to facilitate desired action
}

void testGoToMethod(){
  pros::Gps GpsPrimary(GPS1PORT);

  int randomAngle = 0;
  int xRandom = 0;
  int yRandom = 0; 

  srand((unsigned) pros::millis());
  randomAngle = (rand() % 360) + 1;
  train.faceHeading(randomAngle);
  //This is to prove that the goTo method works no matter the heading of the robot

  srand((unsigned) pros::millis());
  xRandom = (rand() % 1601) - 800;

  pros::delay((rand() % 3 + 1)*1000);

  srand((unsigned) pros::millis());
  yRandom = (rand() % 1601) - 800;
  //generates a set of random coordinates

  master.clear();
  std::cout << "Dx: "+std::to_string(xRandom) << std::endl;
  std::cout << "Dy: "+std::to_string(yRandom) << std::endl;


  //This prints the desired position, that was randomly generated, on the screen of the controller

  train.goToPos(xRandom, yRandom);
  //This sends the robot to a random x and y position


  std::cout << "Ax: "+std::to_string(GpsPrimary.get_status().x * 1000) << std::endl;

  std::cout << "Ay: "+std::to_string(GpsPrimary.get_status().y * 1000) << std::endl;


  //This prints where the robot actually went on the screen of the controller

  return;
}



void disabled() {}


void competition_initialize() {}


void autonomous() {
  pros::Gps GpsPrimary(GPS1PORT);


  // train.goToPos(0, 0, master);
  testGoToMethod();
  
  // master.set_text(1, 1, std::to_string(GpsPrimary.get_heading()));
  // pros::delay(100);

}


void opcontrol() {
  pros::Imu Inertial(INERTIALSENSORPORT);
  pros::Gps GpsPrimaryInit(GPS1PORT);
  float prevTime = 0;
  float prevVelocity = 0;
  while(Inertial.is_calibrating())
  {

  }
  float deltaTime = 0;
	while (true) {
    // deltaTime = pros::millis() - prevTime;
    // float velocity = updateXVelocity(deltaTime, prevVelocity, INERTIALSENSORPORT);
    // prevVelocity = velocity;
    // prevTime = pros::millis();

    // printf("X velocity: %f\n ", velocity); 

    printf("Gps: %f, Imu: %f\n", GpsPrimaryInit.get_heading(), Inertial.get_heading());
    
    
		pros::delay(20);
	}
}
