#pragma once
class Drivetrain {

float wheelDiameter;
float gearRatio;
float wheelCircumference;


//The angle of the vector created when a motor drives forward compared to the heading
int northWheelAngle; 
int southWheelAngle; 
int eastWheelAngle; 
int westWheelAngle;

vex::motor northMotor = NULL;
vex::motor southMotor = NULL;
vex::motor eastMotor = NULL;
vex::motor westMotor = NULL;

public:
Drivetrain(float wheelDiameter, float gearRatio, vex::motor northMotor, vex::motor southMotor, vex::motor eastMotor, vex::motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle);









};