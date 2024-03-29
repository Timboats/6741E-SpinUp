#include "controllers.hxx"
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
void Drivetrain::goToPos(int x, int y, int maxErrParam, int errTimerEnableThreshold, long exitTimer){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    // pros::Imu inertial(inertialPort);
    PIDController<double> gtPid(true);

    const double Kp = 48; //48 is mid
    const double Ki = 0.04; //0.052 and 0.047 is pretty good
    const double Kd = 0;
    const double maxErr = maxErrParam;
    const double windupUpperLimit = 48;
    
    long deltaTime = 0;
    long prevTime = 0;
    long startTime = -1;
    double error = 0;
    double totalVoltage = 0;
    double angleToSetPos = 0;
    double heading = gpsSystem->getHeading();
    double northVoltage = 0;
    double eastVoltage = 0;

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
        gtPid.setError(error);

        totalVoltage = gtPid.calculateOutput(Kp, Ki, Kd, windupUpperLimit, 0, 0);
        angleToSetPos = atan2(y - currentY, x - currentX);

        northVoltage = ((double)(totalVoltage * cos((double)(angleToSetPos - (Simpler::coterminalToStdPos(heading - eastWheelAngle)) * (M_PI/180)))));
        eastVoltage = ((double)(totalVoltage * cos((double)(angleToSetPos - (Simpler::coterminalToStdPos(heading - northWheelAngle)) * (M_PI/180)))));

        northMotor.move_voltage(northVoltage); 
        eastMotor.move_voltage(-eastVoltage);
        southMotor.move_voltage(-northVoltage);
        westMotor.move_voltage(eastVoltage);

        if(((Simpler::abs(error) <= errTimerEnableThreshold) && startTime == -1) || (errTimerEnableThreshold == INFINITY && startTime == -1)){
            startTime = pros::millis();
            printf("started exittime\n");
        }
        if((pros::millis() > startTime+exitTimer && exitTimer != -1) && startTime != -1){
            stopAllDrive();
            return;
        }

        if (Simpler::abs(error) <= maxErr){
            deltaTime = pros::millis() - prevTime;
            if (deltaTime > 250){
                stopAllDrive();
                return;
            }
        } else {
            prevTime = pros::millis();

        } 
  }
  
    
}
void Drivetrain::faceHeading(int heading, int maxErrParam, long exitTimer, bool useInertial){
    pros::Motor northMotor(northMotorPort);
    pros::Motor southMotor(southMotorPort);
    pros::Motor eastMotor(eastMotorPort);
    pros::Motor westMotor(westMotorPort);
    pros::Imu inertial(inertialPort);
    PIDController<double> headPid(true);

    const double Kp = 155; //560 works but oscillates a bit too
    const double Ki = 0.01;
    const double Kd = 75;
    const double windupUpperLimit = 5;

    long deltaTime = 0;
    long prevTime = 0;
    long startTime = -1;
    double error = 0;
    
    const double maxError = maxErrParam;

    double angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    double totalVoltage = 0;


    while(true){
        if(gpsSystem->getHeading() == INFINITY && !useInertial){
            return;
        }

        angleFromSet = !useInertial?((((int)gpsSystem->getHeading() - heading) + 360) % 360):((((int)Simpler::degreeToStdPos(inertial.get_heading()) - heading) + 360) % 360);

        if(angleFromSet > 180){
            error = -(180 - (angleFromSet - 180));
        } else {
            error = angleFromSet;
        }
        headPid.setError(error);

        totalVoltage = headPid.calculateOutput(Kp, Ki, Kd, windupUpperLimit, 0, 0);
        
        northMotor.move_voltage(totalVoltage);
        eastMotor.move_voltage(totalVoltage);
        southMotor.move_voltage(totalVoltage);
        westMotor.move_voltage(totalVoltage);

        if(startTime == -1){
            startTime = pros::millis();
            printf("started exittime\n");
        }

        if((pros::millis() > startTime+exitTimer && exitTimer != -1) && startTime != -1){
            stopAllDrive();
            return;
        }

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