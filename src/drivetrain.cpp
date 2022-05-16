#define _USE_MATH_DEFINES
#include <cmath>
#include "vex.h"
#include "drivetrain.h"
#include "iostream"

Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, vex::motor northMotor, vex::motor southMotor, vex::motor eastMotor, vex::motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle){
    this -> wheelDiameter = wheelDiameter;
    this -> gearRatio = gearRatio;
    this -> wheelCircumference = M_PI * wheelDiameter;
    this -> northWheelAngle = northWheelAngle;
    this -> southWheelAngle = southWheelAngle;
    this -> eastWheelAngle = eastWheelAngle;
    this -> westWheelAngle = westWheelAngle;

    this -> northMotor = northMotor;
    this -> southMotor = southMotor;
    this -> eastMotor = eastMotor;
    this -> westMotor = westMotor;
    
}