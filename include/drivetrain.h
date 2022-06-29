#pragma once
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"
#include <cstddef>

class Drivetrain {

float wheelDiameter;
float gearRatio;
float wheelCircumference;


//The angle of the vector created when a motor drives forward compared to the heading
int northWheelAngle; 
int southWheelAngle; 
int eastWheelAngle; 
int westWheelAngle;



pros::Motor* northMotor;
pros::Motor* southMotor;
pros::Motor* eastMotor;
pros::Motor* westMotor;

pros::Gps* gps1;
pros::Gps* gps2;

public:
Drivetrain(float wheelDiameter, float gearRatio, pros::Motor northMotor, pros::Motor southMotor, pros::Motor eastMotor, pros::Motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle, pros::Gps gps1, pros::Gps gps2 = NULL);
void goToPos(int x, int y);
void stopAllDrive();
void faceHeading(int heading);
float turnToPoint(int x, int y);
void steeringControl(pros::Controller driveController, int storedPercent);
  

};