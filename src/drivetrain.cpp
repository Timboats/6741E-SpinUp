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



Drivetrain::Drivetrain(float wheelDiameter, float gearRatio, unsigned int northMotorPort, unsigned int southMotorPort, unsigned int eastMotorPort, unsigned int westMotorPort, unsigned int northWheelAngle, unsigned int southWheelAngle, unsigned int eastWheelAngle, unsigned int westWheelAngle, unsigned int inertialPort, DualGps* gpsSystem){
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
    this -> gpsSystem = gpsSystem;
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

    float northComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gpsSystem->getHeading()+90)) - northWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float southComponent = -direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gpsSystem->getHeading()+90)) - southWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float eastComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gpsSystem->getHeading()+90)) - eastWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    float westComponent = direction*(desiredMagnitude * (cos((desiredAngle - (Simpler::coterminalToStdPos(gpsSystem->getHeading()+90)) - westWheelAngle) * (M_PI/180)))) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    northMotor.move(northComponent + storedPercent);
    southMotor.move(southComponent + storedPercent);
    eastMotor.move(eastComponent + storedPercent);
    westMotor.move(westComponent + storedPercent);
}
void Drivetrain::goToPos(int x, int y, int maxErrParam){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    // pros::Imu inertial(inertialPort);

    const float Kp = 66; //65 doesnt work
    const float Ki = 0; //9 is pretty good
    const float Kd = 0;
    const int maxErr = maxErrParam;
    const int windupUpperLimit = 8;

    float integral = 0;
    
    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    float totalVoltage = 0;

    double angleToSetPos = 0;
    float heading = gpsSystem->getHeading();

    float northVoltage = 0;
    float eastVoltage = 0;

    double currentX = 0;
    double currentY = 0;
    

    while(true){
        currentX = gpsSystem->getPositions().x * 1000;
        currentY = gpsSystem->getPositions().y * 1000;

        if(gpsSystem->getHeading() == INFINITY){
            return;
        }


        heading = gpsSystem->getHeading();
        


        

        error = Formula::twoCoordDistance(currentX, currentY, x, y);


        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }

        
        

        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));

        angleToSetPos = atan2(y - currentY, x - currentX);


        northVoltage = ((double)(totalVoltage * cos((double)(angleToSetPos - (Simpler::coterminalToStdPos(heading - eastWheelAngle)) * (M_PI/180)))));

        eastVoltage = ((double)(totalVoltage * cos((double)(angleToSetPos - (Simpler::coterminalToStdPos(heading - northWheelAngle)) * (M_PI/180)))));


        
        northMotor.move_voltage(northVoltage); //NE, NW, SW
        eastMotor.move_voltage(-eastVoltage);
        southMotor.move_voltage(-northVoltage);
        westMotor.move_voltage(eastVoltage);

        
        if (Simpler::abs(error) <= maxErr){
            deltaTime = pros::millis() - prevTime;
            if (deltaTime > 250){
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
    const float windupUpperLimit = 2;

    float deltaTime = 0;
    float prevTime = 0;
    float error = 0;
    float prevError = 0;
    
    const float maxError = 1;
    float integral = 0;

    float angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    float totalVoltage = 0;


    while(true){
        if(gpsSystem->getHeading() == INFINITY){
            return;
        }

        angleFromSet = (((int)gpsSystem->getHeading() - heading) + 360) % 360;

        integral = integral + error;

        if (error > windupUpperLimit){
            integral = 0;
        }

        if(angleFromSet > 180){
            error = -(180 - (angleFromSet - 180));
        } else {
            error = angleFromSet;
        }

        totalVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));
        
        northMotor.move_voltage(totalVoltage);
        eastMotor.move_voltage(totalVoltage);
        southMotor.move_voltage(totalVoltage);
        westMotor.move_voltage(totalVoltage);

        if(Simpler::abs(error) <= maxError){
            deltaTime = pros::millis() - prevTime;
            if(deltaTime > 250){
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
void Drivetrain::moveVelocity(int xVelocity, int yVelocity, int heading){
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
    

    float northComponent = -(desiredMagnitude * (cos((desiredAngle - (gpsSystem->getHeading()) - northWheelAngle) * (M_PI/180)))) + heading;
    float southComponent = -(desiredMagnitude * (cos((desiredAngle - (gpsSystem->getHeading()) - southWheelAngle) * (M_PI/180)))) + heading;
    float eastComponent = (desiredMagnitude * (cos((desiredAngle - (gpsSystem->getHeading()) - eastWheelAngle) * (M_PI/180)))) + heading;
    float westComponent = (desiredMagnitude * (cos((desiredAngle - (gpsSystem->getHeading()) - westWheelAngle) * (M_PI/180)))) + heading;

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

    int currentX = gpsSystem->getPositions().x * 1000;
    int currentY = gpsSystem->getPositions().y * 1000;

    desiredAngle = atan2(y - currentY, x - currentX) * (180/M_PI);

    angleFromDesired = (((int)gpsSystem->getHeading() - desiredAngle) + 360) % 360;

    if (angleFromDesired > 180){
        error = -(180 - (angleFromDesired - 180));
    } else {
        error = angleFromDesired;
    }

    motorVoltage = Kp * error;

    return motorVoltage;
}