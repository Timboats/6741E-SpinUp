#include "H-drive.h"

HDrive::HDrive(double wheelDiameter, double gearRatio, unsigned int fLMotorPort, unsigned int fRMotorPort, unsigned int bLMotorPort, unsigned int bRMotorPort, unsigned int inertialPort, DualGps* gpsSystem){
    this -> wheelDiameter = wheelDiameter;
    this -> gearRatio = gearRatio;
    this -> wheelCircumference = M_PI * wheelDiameter;

    this -> fLMotorPort = fLMotorPort;
    this -> fRMotorPort = fRMotorPort;
    this -> bLMotorPort = bLMotorPort;
    this -> bRMotorPort = bRMotorPort;

    this -> gpsSystem = gpsSystem;
    this -> inertialPort = inertialPort;
}
void HDrive::stopAllDrive(){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

    fLMotor.brake();
    fRMotor.brake();
    bLMotor.brake();
    bRMotor.brake();
}
void HDrive::driverCentricSteeringControl(pros::Controller driveController){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);
 
    fRMotor.move(((double)1/127)*((Simpler::sign(-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bRMotor.move(((double)1/127)*((Simpler::sign(-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((-driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    fLMotor.move(((double)1/127)*((Simpler::sign(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bLMotor.move(((double)1/127)*((Simpler::sign(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));

}
void HDrive::moveVelocity(int xVelocity, int yVelocity, int heading){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

}