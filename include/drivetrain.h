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

vex::gps gps1 = NULL;
vex::gps gps2 = NULL;

public:
Drivetrain(float wheelDiameter, float gearRatio, vex::motor northMotor, vex::motor southMotor, vex::motor eastMotor, vex::motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle, vex::gps gps1, vex::gps gps2 = NULL);
void goToPos(int x, int y);
void stopAllDrive();
void faceHeading(int heading);
float turnToPoint(int x, int y);
void steeringControl(vex::controller driveController, int storedPercent);
  

};