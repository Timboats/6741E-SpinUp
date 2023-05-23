#pragma once
#include "gps_wrapper.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/gps.hpp"
#include <cstddef>
#include <cmath>
#include "controllers.hxx"
#include "pros/rtos.hpp"


class HDrive {
    double wheelDiameter;
    double gearRatio;
    double wheelCircumference;

    unsigned int driveMotor1Port;
    unsigned int driveMotor2Port;
    unsigned int driveMotor3Port;
    unsigned int driveMotor4Port;
    unsigned int driveMotor5Port;
    unsigned int driveMotor6Port;

    DualGps* gpsSystem;
    unsigned int inertialPort;

    long drivePrevTime = 0;
    int prevMotorVelocity = 0;
    unsigned int maxYChange = 0;

    public:

    HDrive(double wheelDiameter, double gearRatio, unsigned int driveMotor1, unsigned int driveMotor2, unsigned int driveMotor3, unsigned int driveMotor4, unsigned int driveMotor5, unsigned int driveMotor6, unsigned int inertialPort, DualGps* gpsSystem);
    void stopAllDrive();
    void driverCentricSteeringControl(pros::Controller driveController, double maxYChange, double maxYChangeNeg);
    void fieldCentricSteeringControl(pros::Controller driveController, int storedPercent, int direction);
    void moveVelocity(int yVelocity, int heading);
    void faceHeading(int heading, int maxErrParam = 2, long exitTimer = 5000, bool useInertial = false);
    void goToPos(int x, int y, int maxErrParam = 45, int errTimerEnableThreshold = 70, long exitTimer = 5000); //inf errTimer makes time enabled at all times, -1 exitTimer disables timer as a whole
    void moveDistance(int d, int maxErrParam = 30);


};