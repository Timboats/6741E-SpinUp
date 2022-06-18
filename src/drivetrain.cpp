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

    if(gps2.installed()){
        this -> gps2 = gps2;
    }


}
//test
void Drivetrain::stopAllDrive(){
    northMotor.stop();
    southMotor.stop();
    eastMotor.stop();
    westMotor.stop();
}
void printToController (float value, int xPos, int yPos){
  Controller1.Screen.setCursor(xPos,yPos);
  Controller1.Screen.clearScreen();
  Controller1.Screen.print(value);
}
void Drivetrain::steeringControl(vex::controller driveController, int storedPercent){
    northMotor.spin(forward, -(driveController.Axis3.position() * (cos(Simpler::abs((90 - (Simpler::degreeToStdPos(GPS16.heading())) - 45) * (M_PI/180))))) - (driveController.Axis4.position() * (cos(Simpler::abs((90 + (Simpler::degreeToStdPos(GPS16.heading())) - 45) * (M_PI/180))))) + Controller1.Axis1.position() + storedPercent, pct);
    southMotor.spin(forward, -(driveController.Axis3.position() * (cos(Simpler::abs((90 - (Simpler::degreeToStdPos(GPS16.heading())) - 225) * (M_PI/180))))) - (driveController.Axis4.position() * (cos(Simpler::abs((90 + (Simpler::degreeToStdPos(GPS16.heading())) - 225) * (M_PI/180))))) + Controller1.Axis1.position() + storedPercent, pct);
    eastMotor.spin(forward, (driveController.Axis3.position() * (cos(Simpler::abs((90 - (Simpler::degreeToStdPos(GPS16.heading())) - 135) * (M_PI/180))))) - (driveController.Axis4.position() * (cos(Simpler::abs((90 + (Simpler::degreeToStdPos(GPS16.heading())) - 135) * (M_PI/180))))) + Controller1.Axis1.position() + storedPercent, pct);
    westMotor.spin(forward, (driveController.Axis3.position() * (cos(Simpler::abs((90 - (Simpler::degreeToStdPos(GPS16.heading())) - 315) * (M_PI/180))))) - (driveController.Axis4.position() * (cos(Simpler::abs((90 + (Simpler::degreeToStdPos(GPS16.heading())) - 315) * (M_PI/180))))) + Controller1.Axis1.position() + storedPercent, pct);
}
void Drivetrain::goToPos(int x, int y){
    const float Kp = 101;
    const float Ki = 0.2;
    const float Kd = 0;
    const int maxErr = 15;
    const int windupUpperLimit = 2;

    float integral = 0;


    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    float totalVoltage = 0;

    double angleToSetPos = 0;
    int heading = gps1.heading();

    float northVoltage = 0;
    float eastVoltage = 0;

    int currentX = gps1.xPosition(mm);
    int currentY = gps1.yPosition(mm);

    while(true){
        currentX = gps1.xPosition(mm);
        currentY = gps1.yPosition(mm);
        heading = gps1.heading();

        error = Formula::twoCoordDistance(currentX, currentY, x, y);

        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }


        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));
        angleToSetPos = atan2(y - currentY, x - currentX);

        northVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - 225)) * (M_PI/180))))));

        eastVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - 135)) * (M_PI/180))))));
        
        northMotor.spin(forward, (northVoltage), vex::voltageUnits::mV);
        eastMotor.spin(forward, (eastVoltage), vex::voltageUnits::mV);
        southMotor.spin(forward, -(northVoltage), vex::voltageUnits::mV);
        westMotor.spin(forward, -(eastVoltage), vex::voltageUnits::mV);
        
        


        if (Simpler::abs(error) <= maxErr){
            deltaTime = Brain.Timer.time(msec) - prevTime;
            if (deltaTime > 500){
                stopAllDrive();
                return;
            }
        } else {
            prevTime = Brain.Timer.time(msec);

        } 

        prevError = error;
  }
  
    
}
void Drivetrain::faceHeading(int heading){
    const float Kp = 250;
    const float Ki = 0;
    const float Kd = 0;

    float deltaTime = 0;
    float prevTime = 0;
    int error = 0;

    int angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    float totalVoltage = 0;

    while(true){
        angleFromSet = ((Simpler::degreeToStdPos(gps1.heading()) - heading) + 360) % 360;

        if(angleFromSet > 180){
            error = -(180 - (angleFromSet - 180));
        } else {
            error = angleFromSet;
        }

        totalVoltage = Kp * error;

        northMotor.spin(forward, totalVoltage, vex::voltageUnits::mV);
        eastMotor.spin(forward, totalVoltage, vex::voltageUnits::mV);
        southMotor.spin(forward, totalVoltage, vex::voltageUnits::mV);
        westMotor.spin(forward, totalVoltage, vex::voltageUnits::mV);

        if(Simpler::abs(error) <= 1){
            deltaTime = Brain.Timer.time(msec) - prevTime;
            if(deltaTime > 500){
                stopAllDrive();
                return;
            }
        }
        else {
            prevTime = Brain.Timer.time(msec);
        }
    }
}
float Drivetrain::turnToPoint(int x, int y){
    const float Kp = 1;
    const float Ki = 0;
    const float Kd = 0;

    float deltaTime = 0;
    float prevTime = 0;
    int error = 0;

    float motorVoltage = 0;

    int desiredAngle = 0;
    int angleFromDesired = 0;

    int currentX = gps1.xPosition(mm);
    int currentY = gps1.yPosition(mm);

    desiredAngle = atan2(y - currentY, x - currentX) * (180/M_PI);

    angleFromDesired = ((Simpler::degreeToStdPos(gps1.heading()) - desiredAngle) + 360) % 360;

    if (angleFromDesired > 180){
        error = -(180 - (angleFromDesired - 180));
    } else {
        error = angleFromDesired;
    }

    motorVoltage = Kp * error;

    return motorVoltage;
}