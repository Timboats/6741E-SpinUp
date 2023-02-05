#pragma once
#include "gps_wrapper.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"
#include <cstddef>

class HDrive {
    double wheelDiameter;
    double gearRatio;
    double wheelCircumference;

    unsigned int fLMotorPort;
    unsigned int fRMotorPort;
    unsigned int bLMotorPort;
    unsigned int bRMotorPort;

    DualGps* gpsSystem;
    unsigned int inertialPort;

    public:

    HDrive(double wheelDiameter, double gearRatio, unsigned int fLMotorPort, unsigned int fRMotorPort, unsigned int bLMotorPort, unsigned int bRMotorPort, unsigned int inertialPort, DualGps* gpsSystem);
    void stopAllDrive();
    void driverCentricSteeringControl(pros::Controller driveController);
    void moveVelocity(int xVelocity, int yVelocity, int heading);

};