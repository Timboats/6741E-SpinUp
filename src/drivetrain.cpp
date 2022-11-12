#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <string>
#include "globals.hpp"
#include <cmath>
#include "drivetrain.h"
#include "iostream"
#include "mathlib.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"



Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, unsigned int northMotorPort, unsigned int southMotorPort, unsigned int eastMotorPort, unsigned int westMotorPort, unsigned int northWheelAngle, unsigned int southWheelAngle, unsigned int eastWheelAngle, unsigned int westWheelAngle, unsigned int inertialPort, GpsWrapper* gps1, int gps2Port){
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
    this -> gps1 = gps1;
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
void Drivetrain::driverCentricSteeringControl(pros::Controller driveController){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);

    double desiredAngle = atan2(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * (180/M_PI); 
    double desiredMagnitude = sqrt(pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2) + pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2));

    float northComponent = -(desiredMagnitude * (cos((desiredAngle - northWheelAngle - 90) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float southComponent = -(desiredMagnitude * (cos((desiredAngle - southWheelAngle - 90) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float eastComponent = (desiredMagnitude * (cos((desiredAngle - eastWheelAngle - 90) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float westComponent = (desiredMagnitude * (cos((desiredAngle - westWheelAngle - 90) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    northMotor.move(northComponent);
    southMotor.move(southComponent);
    eastMotor.move(eastComponent);
    westMotor.move(westComponent);
}
void Drivetrain::fieldCentricSteeringControl(pros::Controller driveController, int storedPercent, int direction){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    // pros::Imu inertial(inertialPort);


    double desiredAngle = 0;
    double desiredMagnitude = 0;

    desiredAngle = atan2(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * (180/M_PI); 
    desiredMagnitude = sqrt(pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2) + pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2));

    float northComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gps1->getHeading()+90)) - northWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float southComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gps1->getHeading()+90)) - southWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float eastComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gps1->getHeading()+90)) - eastWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float westComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gps1->getHeading()+90)) - westWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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
    // pros::Imu inertial(inertialPort);

    const float Kp = 70; //61 doesnt work
    const float Ki = 0; //9 is pretty good
    const float Kd = 0;
    const int maxErr = 20;
    const int windupUpperLimit = 8;

    float integral = 0;
    
    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    float totalVoltage = 0;

    double angleToSetPos = 0;
    int heading = gps1->getHeading();
    int initHeading = heading; //Might have to be removed later

    float northVoltage = 0;
    float eastVoltage = 0;

    int currentX = 0;
    int currentY = 0;
    

    while(true){
        currentX = gps1->getPositions().x * 1000;
        currentY = gps1->getPositions().y * 1000;

        if(gps1->getHeading() == INFINITY){
            return;
        }


        heading = gps1->getHeading();


        

        error = Formula::twoCoordDistance(currentX, currentY, x, y);


        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }

        
        

        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));

        angleToSetPos = atan2(y - currentY, x - currentX);


        northVoltage = ((float)(totalVoltage * cos((double)((angleToSetPos - (Simpler::degreeToStdPos(heading - eastWheelAngle)) * (M_PI/180))))));

        eastVoltage = ((float)(totalVoltage * cos((double)((angleToSetPos - (Simpler::degreeToStdPos(heading - northWheelAngle)) * (M_PI/180))))));


        
        northMotor.move_voltage(northVoltage); 
        eastMotor.move_voltage(eastVoltage);
        southMotor.move_voltage(-northVoltage);
        westMotor.move_voltage(-eastVoltage);

        
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
    // pros::Imu inertial(inertialPort);

    const float Kp = 560; //560 works but oscillates a bit too
    const float Ki = 0;
    const float Kd = 0;

    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    
    float maxError = 1;

    float angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    float totalVoltage = 0;


    while(true){
        if(gps1->getHeading() == INFINITY){
            return;
        }

        angleFromSet = (((int)gps1->getHeading() - heading) + 360) % 360;

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

        if(Simpler::abs(error) <= maxError){
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
void Drivetrain::moveVelocity(int xVelocity, int yVelocity, unsigned int heading){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);

    double desiredAngle = 0;
    double desiredMagnitude = 0;

    desiredAngle = atan2(yVelocity, xVelocity) * (180/M_PI); 
    desiredMagnitude = sqrt(pow(yVelocity, 2) + pow(xVelocity, 2));

    float northComponent = -(desiredMagnitude * (cos((desiredAngle - 90 - northWheelAngle) * (M_PI/180)))) + heading;
    float southComponent = -(desiredMagnitude * (cos((desiredAngle - 90 - southWheelAngle) * (M_PI/180)))) + heading;
    float eastComponent = (desiredMagnitude * (cos((desiredAngle - 90 - eastWheelAngle) * (M_PI/180)))) + heading;
    float westComponent = (desiredMagnitude * (cos((desiredAngle - 90 - westWheelAngle) * (M_PI/180)))) + heading;

    northMotor.move(northComponent);
    southMotor.move(southComponent);
    eastMotor.move(eastComponent);
    westMotor.move(westComponent);
}
void Drivetrain::moveVelocityFieldCentric(int xVelocity, int yVelocity, unsigned int heading){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);

    double desiredAngle = 0;
    double desiredMagnitude = 0;

    desiredAngle = atan2(yVelocity, xVelocity) * (180/M_PI); 
    desiredMagnitude = sqrt(pow(yVelocity, 2) + pow(xVelocity, 2));

    // float headingControl = driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    

    float northComponent = -(desiredMagnitude * (cos((desiredAngle - (gps1->getHeading()) - northWheelAngle) * (M_PI/180)))) + heading;
    float southComponent = -(desiredMagnitude * (cos((desiredAngle - (gps1->getHeading()) - southWheelAngle) * (M_PI/180)))) + heading;
    float eastComponent = (desiredMagnitude * (cos((desiredAngle - (gps1->getHeading()) - eastWheelAngle) * (M_PI/180)))) + heading;
    float westComponent = (desiredMagnitude * (cos((desiredAngle - (gps1->getHeading()) - westWheelAngle) * (M_PI/180)))) + heading;

    northMotor.move(northComponent);
    southMotor.move(southComponent);
    eastMotor.move(eastComponent);
    westMotor.move(westComponent);




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

    int currentX = gps1->getPositions().x * 1000;
    int currentY = gps1->getPositions().y * 1000;

    desiredAngle = atan2(y - currentY, x - currentX) * (180/M_PI);

    angleFromDesired = (((int)gps1->getHeading() - desiredAngle) + 360) % 360;

    if (angleFromDesired > 180){
        error = -(180 - (angleFromDesired - 180));
    } else {
        error = angleFromDesired;
    }

    motorVoltage = Kp * error;

    return motorVoltage;
}