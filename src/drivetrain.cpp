#define _USE_MATH_DEFINES
#include <cmath>
#include "vex.h"
#include "drivetrain.h"
#include "iostream"
#include "mathlib.h"

Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, vex::motor northMotor, vex::motor southMotor, vex::motor eastMotor, vex::motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle, vex::gps gps1, vex::gps gps2){
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
    this -> gps1 = gps1;
    /*
    Figure this out later

    if(gps2 != 0){
        this -> gps2 = gps2;
    }*/


}
void Drivetrain::goToPos(int x, int y){
  const float Kp = 750;
  const float Ki = 0;
  const float Kd = 0;

  float deltaTime = 0;
  float prevTime = 0;
  int error = 0;
  float totalVoltage = 0;

  float angleToSetPos = 0;

  float northVoltage = 0;
  float eastVoltage = 0;

  while(true){
      //Next time working define a fuction that does the distance formula
      

  }
  
    
}
