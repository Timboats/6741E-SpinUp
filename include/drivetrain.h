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



int northMotorPort;
int southMotorPort;
int eastMotorPort;
int westMotorPort;

int gps1Port;
int gps2Port;

public:
Drivetrain(float wheelDiameter, float gearRatio, int northMotorPort, int southMotorPort, int eastMotorPort, int westMotorPort, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle, int gps1Port, int gps2Port = -1);
void goToPos(int x, int y);
void stopAllDrive();
void faceHeading(int heading);
float turnToPoint(int x, int y);
void steeringControl(pros::Controller driveController, int storedPercent);
  

};