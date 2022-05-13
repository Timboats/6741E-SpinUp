#pragma once
class Drivetrain {

float wheelDiameter;
float gearRatio;
const float wheelCircumference;


//The angle of the vector created when a motor drives forward compared to the heading
const int northWheelAngle; 
const int southWheelAngle; 
const int eastWheelAngle; 
const int westWheelAngle;

vex::motor northMotor = NULL;
vex::motor southMotor = NULL;
vex::motor eastMotor = NULL;
vex::motor westMotor = NULL;

public:
Drivetrain(float wheelDiameter, float gearRatio, vex::motor northMotor, vex::motor southMotor, vex::motor eastMotor, vex::motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle);









};