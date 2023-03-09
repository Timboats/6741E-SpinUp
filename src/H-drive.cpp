#include "H-drive.h"
#include "controllers.hxx"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>



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
void HDrive::driverCentricSteeringControl(pros::Controller driveController, double maxYChange, double maxYChangeNeg){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);
    
    int yInput = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    long deltaTime = pros::millis() - drivePrevTime;
    int deltaY = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - prevMotorVelocity;

    if((double)deltaY/deltaTime > maxYChange){
        yInput = prevMotorVelocity + maxYChange;
    }

    if((double)deltaY/deltaTime < maxYChangeNeg){
        yInput = prevMotorVelocity - maxYChangeNeg;
    }

    fRMotor.move(((double)1/127)*((Simpler::sign(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bRMotor.move(((double)1/127)*((Simpler::sign(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    fLMotor.move(((double)1/127)*((Simpler::sign(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    bLMotor.move(((double)1/127)*((Simpler::sign(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));

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

    const double Kp = 190; //155 works but oscillates a bit too
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
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);
    // pros::Imu inertial(inertialPort);
    PIDController<double> gtPid(true);
    PIDController<double> angPid(true);

    const double l_Kp = 48; //48 is mid
    const double l_Ki = 0.04; //0.052 and 0.047 is pretty good
    const double l_Kd = 0;

    const double a_Kp = 147;
    const double a_Ki = 10;
    const double a_windupUpperLimit = INFINITY;
    const double maxErr = maxErrParam;
    const double windupUpperLimit = 48;
    
    long deltaTime = 0;
    long prevTime = 0;
    long startTime = -1;
    double linError = 0;
    double targetAng = 0;
    double angError = 0;
    double heading = gpsSystem->getHeading();

    double currentX = 0;
    double currentY = 0;
    

    while(true){
        currentX = gpsSystem->getPositions().x * 1000;
        currentY = gpsSystem->getPositions().y * 1000;

        if(gpsSystem->getHeading() == INFINITY){
            return;
        }

        heading = gpsSystem->getHeading();

        

        linError = Formula::twoCoordDistance(currentX, currentY, x, y);
        gtPid.setError(linError);

        targetAng = atan2(y - currentY, x - currentX);

        if(targetAng < 0){
            targetAng+=(2*M_PI);
        }

        targetAng = targetAng * (180/M_PI);

        angError = ((int)(heading - targetAng) + 360) % 360; //might have to use degreetostdpos here for targetAng

        if(angError > 180){
            angError = -(180 - (angError - 180));
        }


        
        if(pros::millis() % 1000 == 0){
            printf("linErr: %f\n", linError);
        }
        

        angPid.setError(angError);

        double linearAppliedVoltage = gtPid.calculateOutput(l_Kp, l_Ki, l_Kd, windupUpperLimit, 0, 0);
        double angularAppliedVoltage = angPid.calculateOutput(a_Kp, a_Ki, 0, a_windupUpperLimit, 0, 0);

        linearAppliedVoltage = 0;

        fLMotor.move_voltage((linearAppliedVoltage+angularAppliedVoltage));
        fRMotor.move_voltage((linearAppliedVoltage-angularAppliedVoltage));
        bLMotor.move_voltage((linearAppliedVoltage+angularAppliedVoltage));
        bRMotor.move_voltage((linearAppliedVoltage-angularAppliedVoltage));

        if(((Simpler::abs(linError) <= errTimerEnableThreshold) && startTime == -1) || (errTimerEnableThreshold == INFINITY && startTime == -1)){
            startTime = pros::millis();
            printf("started exittime\n");
        }
        if((pros::millis() > startTime+exitTimer && exitTimer != -1) && startTime != -1){
            stopAllDrive();
            return;
        }

        if (Simpler::abs(linError) <= maxErr){
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

void HDrive::moveDistance(int d, int maxErrParam){
    pros::Motor fLMotor(fLMotorPort);
    pros::Motor fRMotor(fRMotorPort);
    pros::Motor bLMotor(bLMotorPort);
    pros::Motor bRMotor(bRMotorPort);

    pros::Imu inertial(inertialPort);

    const double l_Kp = 28; //48 is mid
    const double l_Ki = 0; //0.04 and 0.047 is pretty good
    const double l_Kd = 0;
    const double l_windupUpperLimit = 5;

    const double a_Kp = 190;
    const double a_Ki = 0.025; //0.01
    const double a_windupUpperLimit = 5;

    PIDController<double> headPid(true);
    PIDController<double> linearPid(false);

    const double lockHeading = inertial.get_heading();

    double angError = 0; 

    long deltaTime = 0;
    long prevTime = 0;

    fLMotor.tare_position();
    fRMotor.tare_position();
    bLMotor.tare_position();
    bRMotor.tare_position();

    fRMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    fLMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    bRMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    bLMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

    while(true){
        if(inertial.get_heading() == INFINITY){
            return;
        }

        double angleFromSet = lockHeading - inertial.get_heading();

        double averageDistance = (((fLMotor.get_position()+fRMotor.get_position()+bRMotor.get_position()+bLMotor.get_position())/4)*gearRatio*M_PI*(wheelDiameter*25.4));

        if(angleFromSet > 180 || angleFromSet < -180){
            angError = -1*Simpler::sign(angleFromSet)*(360 - Simpler::abs(angleFromSet));
        } else {
            angError = angleFromSet;
        }

        headPid.setError(angError);

        double angVoltage = headPid.calculateOutput(a_Kp, a_Ki, 0, a_windupUpperLimit, 0, 0);
        double linearVoltage = linearPid.calculateOutput(l_Kp, l_Ki, l_Kd, l_windupUpperLimit, d, averageDistance);
        
        fLMotor.move_voltage(linearVoltage+angVoltage);
        fRMotor.move_voltage(linearVoltage-angVoltage);
        bLMotor.move_voltage(linearVoltage+angVoltage);
        bRMotor.move_voltage(linearVoltage-angVoltage);
        

        if(Simpler::abs(linearPid.getError()) <= maxErrParam){
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


    


