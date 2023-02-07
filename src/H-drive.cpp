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
void HDrive::driverCentricSteeringControl(pros::Controller driveController, int damperCoefficien){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

    long deltaTime = pros::millis() - drivePrevTime;
    int deltaY = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - prevMotorVelocity;

    double yInput = (double)deltaY/deltaTime;
 
    fRMotor.move(((double)1/127)*((Simpler::sign(damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bRMotor.move(((double)1/127)*((Simpler::sign(damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((damperCoefficien*yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    fLMotor.move(((double)1/127)*((Simpler::sign(damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bLMotor.move(((double)1/127)*((Simpler::sign(damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((damperCoefficien*yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));

    drivePrevTime = pros::millis();
    prevMotorVelocity = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

}
void HDrive::moveVelocity(int yVelocity, int heading){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

    fRMotor.move(yVelocity - heading);
    bRMotor.move(yVelocity - heading);
    fLMotor.move(yVelocity + heading);
    bLMotor.move(yVelocity + heading);

}
void HDrive::faceHeading(int heading, int maxErrParam, long exitTimer, bool useInertial){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

    pros::Imu inertial(inertialPort);
    PIDController<double> headPid(true);

    const double Kp = 195; //155 works but oscillates a bit too
    const double Ki = 0;
    const double Kd = 0;
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
        
        fLMotor.move_voltage(totalVoltage);
        fRMotor.move_voltage(-totalVoltage);
        bLMotor.move_voltage(totalVoltage);
        bRMotor.move_voltage(-totalVoltage);

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
void HDrive::fieldCentricSteeringControl(pros::Controller driveController, int storedPercent, int direction){

}
void HDrive::goToPos(int x, int y, int maxErrParam, int errTimerEnableThreshold, long exitTimer){

}