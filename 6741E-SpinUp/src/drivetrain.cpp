#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include "drivetrain.h"
#include "iostream"
#include "mathlib.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"



Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, pros::Motor northMotor, pros::Motor southMotor, pros::Motor eastMotor, pros::Motor westMotor, int northWheelAngle, int southWheelAngle, int eastWheelAngle, int westWheelAngle, pros::Gps gps1, pros::Gps gps2){
    this -> wheelDiameter = wheelDiameter;
    this -> gearRatio = gearRatio;
    this -> wheelCircumference = M_PI * wheelDiameter;
    this -> northWheelAngle = northWheelAngle;
    this -> southWheelAngle = southWheelAngle;
    this -> eastWheelAngle = eastWheelAngle;
    this -> westWheelAngle = westWheelAngle;

    this -> northMotor = &northMotor;
    this -> southMotor = &southMotor;
    this -> eastMotor = &eastMotor;
    this -> westMotor = &westMotor;
    this -> gps1 = &gps1;    

    // if(gps2 == ){
    //     this -> gps2 = gps2;
    // }


}

void Drivetrain::stopAllDrive(){
    northMotor->brake();
    southMotor->brake();
    eastMotor->brake();
    westMotor->brake();
}
void Drivetrain::steeringControl(pros::Controller driveController, int storedPercent){
    double desiredAngle = 0;
    double desiredMagnitude = 0;

    desiredAngle = atan2(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * (180/M_PI); 
    desiredMagnitude = sqrt(pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2) + pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2));

    float northComponent = -(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(gps1->get_heading())) - northWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float southComponent = -(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(gps1->get_heading())) - southWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float eastComponent = (desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(gps1->get_heading())) - eastWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float westComponent = (desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(gps1->get_heading())) - westWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    northMotor->move(northComponent + storedPercent);
    southMotor->move(southComponent + storedPercent);
    eastMotor->move(eastComponent + storedPercent);
    westMotor->move(westComponent + storedPercent);
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
    int heading = gps1->get_heading();

    float northVoltage = 0;
    float eastVoltage = 0;

    int currentX = gps1->get_status().x * 1000;
    int currentY = gps1->get_status().y * 1000;

    while(true){
        currentX = gps1->get_status().x * 1000;
        currentY = gps1->get_status().y * 1000;
        heading = gps1->get_heading();

        error = Formula::twoCoordDistance(currentX, currentY, x, y);

        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }


        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));
        angleToSetPos = atan2(y - currentY, x - currentX);

        northVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - 225)) * (M_PI/180))))));

        eastVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - 135)) * (M_PI/180))))));
        
        northMotor->move_voltage(northVoltage);
        eastMotor->move_voltage(eastVoltage);
        southMotor->move_voltage(-northVoltage);
        westMotor->move_voltage(-eastVoltage);
        
        


        if (Simpler::abs(error) <= maxErr){
            deltaTime = pros::millis() - prevTime;
            if (deltaTime > 500){
                stopAllDrive();
                return;
            }
        } else {
            prevTime = pros::millis();

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
        angleFromSet = ((Simpler::degreeToStdPos(gps1->get_heading()) - heading) + 360) % 360;

        if(angleFromSet > 180){
            error = -(180 - (angleFromSet - 180));
        } else {
            error = angleFromSet;
        }

        totalVoltage = Kp * error;

        northMotor->move_voltage(totalVoltage);
        eastMotor->move_voltage(totalVoltage);
        southMotor->move_voltage(totalVoltage);
        westMotor->move_voltage(totalVoltage);

        if(Simpler::abs(error) <= 1){
            deltaTime = pros::millis() - prevTime;
            if(deltaTime > 500){
                stopAllDrive();
                return;
            }
        }
        else {
            prevTime = pros::millis();
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

    int currentX = gps1->get_status().x * 1000;
    int currentY = gps1->get_status().y * 1000;

    desiredAngle = atan2(y - currentY, x - currentX) * (180/M_PI);

    angleFromDesired = ((Simpler::degreeToStdPos(gps1->get_heading()) - desiredAngle) + 360) % 360;

    if (angleFromDesired > 180){
        error = -(180 - (angleFromDesired - 180));
    } else {
        error = angleFromDesired;
    }

    motorVoltage = Kp * error;

    return motorVoltage;
}