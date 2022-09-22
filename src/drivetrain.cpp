#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include "drivetrain.h"
#include "iostream"
#include "mathlib.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"



Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, unsigned int northMotorPort, unsigned int southMotorPort, unsigned int eastMotorPort, unsigned int westMotorPort, unsigned int northWheelAngle, unsigned int southWheelAngle, unsigned int eastWheelAngle, unsigned int westWheelAngle, unsigned int inertialPort, unsigned int gps1Port, int gps2Port){
    this -> wheelDiameter = wheelDiameter;
    this -> gearRatio = gearRatio;
    this -> wheelCircumference = M_PI * wheelDiameter;
    this -> northWheelAngle = northWheelAngle;
    this -> southWheelAngle = southWheelAngle;
    this -> eastWheelAngle = eastWheelAngle;
    this -> westWheelAngle = westWheelAngle;

    this -> northMotorPort = northMotorPort;
    this -> southMotorPort = southMotorPort;
    this -> eastMotorPort = eastMotorPort;
    this -> westMotorPort = westMotorPort;
    this -> gps1Port = gps1Port;
    this -> inertialPort = inertialPort;

    
    // if(gps2 == ){
    //     this -> gps2 = gps2;
    // }


}
void Drivetrain::stopAllDrive(){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);

    northMotor.brake();
    southMotor.brake();
    eastMotor.brake();
    westMotor.brake();
}
void Drivetrain::steeringControl(pros::Controller driveController, int storedPercent, int direction){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    pros::Imu inertial(inertialPort);


    double desiredAngle = 0;
    double desiredMagnitude = 0;

    desiredAngle = atan2(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * (180/M_PI); 
    desiredMagnitude = sqrt(pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2) + pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2));

    float northComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(inertial.get_heading())) - northWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float southComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(inertial.get_heading())) - southWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float eastComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(inertial.get_heading())) - eastWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float westComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::degreeToStdPos(inertial.get_heading())) - westWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    northMotor.move(northComponent + storedPercent);
    southMotor.move(southComponent + storedPercent);
    eastMotor.move(eastComponent + storedPercent);
    westMotor.move(westComponent + storedPercent);
}
void Drivetrain::goToPos(int x, int y){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    pros::Gps gps1(gps1Port);

    const float Kp = 54.96; //was 61
    const float Ki = 0.03; //9 is pretty good
    const float Kd = 0;
    const int maxErr = 10;
    const int windupUpperLimit = 8;

    float integral = 0;
    
    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    float totalVoltage = 0;

    double angleToSetPos = 0;
    int heading = gps1.get_heading();
    int initHeading = heading; //Might have to be removed later

    float northVoltage = 0;
    float eastVoltage = 0;

    int currentX = 0;
    int currentY = 0;
    

    while(true){
        currentX = gps1.get_status().x * 1000;
        currentY = gps1.get_status().y * 1000;
        heading = gps1.get_heading();


        

        error = Formula::twoCoordDistance(currentX, currentY, x, y);


        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }

        
        float poopityScoop = turnToPoint(cos(initHeading)+currentX, sin(initHeading)+currentY); //Might have to be removed later

        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));

        angleToSetPos = atan2(y - currentY, x - currentX);


        northVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - eastWheelAngle)) * (M_PI/180))))));

        eastVoltage = ((int)(totalVoltage * cos((double)(Simpler::abs(angleToSetPos - (Simpler::degreeToStdPos(heading - northWheelAngle)) * (M_PI/180))))));


        
        northMotor.move_voltage(northVoltage+poopityScoop); //and obv remove these poopityScoops when the ones above are removed
        eastMotor.move_voltage(eastVoltage+poopityScoop);
        southMotor.move_voltage(-northVoltage+poopityScoop);
        westMotor.move_voltage(-eastVoltage+poopityScoop);
        
        
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
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    pros::Gps gps1(gps1Port);

    const float Kp = 500;
    const float Ki = 0;
    const float Kd = 0;

    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;

    int angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    float totalVoltage = 0;


    while(true){
        angleFromSet = ((Simpler::degreeToStdPos(gps1.get_heading()) - heading) + 360) % 360;

        if(angleFromSet > 180){
            error = -(180 - (angleFromSet - 180));
        } else {
            error = angleFromSet;
        }

        totalVoltage = (Kp * error) + (Kd*(error - prevError));

        northMotor.move_voltage(totalVoltage);
        eastMotor.move_voltage(totalVoltage);
        southMotor.move_voltage(totalVoltage);
        westMotor.move_voltage(totalVoltage);

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
    prevError = error;
}
float Drivetrain::turnToPoint(int x, int y){
    pros::Gps gps1(gps1Port);

    const float Kp = 1;
    const float Ki = 0;
    const float Kd = 0;

    float deltaTime = 0;
    float prevTime = 0;
    int error = 0;

    float motorVoltage = 0;

    int desiredAngle = 0;
    int angleFromDesired = 0;

    int currentX = gps1.get_status().x * 1000;
    int currentY = gps1.get_status().y * 1000;

    desiredAngle = atan2(y - currentY, x - currentX) * (180/M_PI);

    angleFromDesired = ((Simpler::degreeToStdPos(gps1.get_heading()) - desiredAngle) + 360) % 360;

    if (angleFromDesired > 180){
        error = -(180 - (angleFromDesired - 180));
    } else {
        error = angleFromDesired;
    }

    motorVoltage = Kp * error;

    return motorVoltage;
}