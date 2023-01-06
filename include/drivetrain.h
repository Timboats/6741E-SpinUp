#pragma once
#include "gps_wrapper.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"
#include <cstddef>

class Drivetrain {

float wheelDiameter;
float gearRatio;
float wheelCircumference;


//The angle of the vector created when a motor drives forward compared to the heading
unsigned int northWheelAngle; 
unsigned int southWheelAngle; 
unsigned int eastWheelAngle; 
unsigned int westWheelAngle;



unsigned int northMotorPort;
unsigned int southMotorPort;
unsigned int eastMotorPort;
unsigned int westMotorPort;

DualGps* gpsSystem;
unsigned int inertialPort;

public:
Drivetrain(float wheelDiameter, float gearRatio, unsigned int northMotorPort, unsigned int southMotorPort, unsigned int eastMotorPort, unsigned int westMotorPort, unsigned int northWheelAngle, unsigned int southWheelAngle, unsigned int eastWheelAngle, unsigned int westWheelAngle, unsigned int inertialPort, DualGps* gpsSystem);
void goToPos(int x, int y, int maxErrParam = 45);
void stopAllDrive();
void faceHeading(int heading);
float turnToPoint(int x, int y);
void driverCentricSteeringControl(pros::Controller driveController);
void fieldCentricSteeringControl(pros::Controller driveController, int storedPercent, int direction);
void moveVelocity(int xVelocity, int yVelocity, int heading);
void moveVelocityFieldCentric(int xVelocity, int yVelocity, unsigned int heading);

  

};