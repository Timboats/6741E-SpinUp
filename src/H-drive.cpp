#include "H-drive.h"
#include "controllers.hxx"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>



HDrive::HDrive(double wheelDiameter, double gearRatio, unsigned int driveMotor1, unsigned int driveMotor2, unsigned int driveMotor3, unsigned int driveMotor4, unsigned int driveMotor5, unsigned int driveMotor6, unsigned int inertialPort, DualGps* gpsSystem){
    this -> wheelDiameter = wheelDiameter;
    this -> gearRatio = gearRatio;
    this -> wheelCircumference = M_PI * wheelDiameter;

    this -> driveMotor1Port = driveMotor1;
    this -> driveMotor2Port = driveMotor2;
    this -> driveMotor3Port = driveMotor3;
    this -> driveMotor4Port = driveMotor4;
    this -> driveMotor5Port = driveMotor5;
    this -> driveMotor6Port = driveMotor6;

    this -> gpsSystem = gpsSystem;
    this -> inertialPort = inertialPort;
}
void HDrive::stopAllDrive(){
    pros::Motor driveMotor1(driveMotor1Port);
    pros::Motor driveMotor2(driveMotor2Port);
    pros::Motor driveMotor3(driveMotor3Port);
    pros::Motor driveMotor4(driveMotor4Port);
    pros::Motor driveMotor5(driveMotor5Port);
    pros::Motor driveMotor6(driveMotor6Port);

    driveMotor1.brake();
    driveMotor2.brake();
    driveMotor3.brake();
    driveMotor4.brake();
    driveMotor5.brake();
    driveMotor6.brake();
}
void HDrive::driverCentricSteeringControl(pros::Controller driveController, double maxYChange, double maxYChangeNeg){
    pros::Motor driveMotor1(driveMotor1Port);
    pros::Motor driveMotor2(driveMotor2Port);
    pros::Motor driveMotor3(driveMotor3Port);
    pros::Motor driveMotor4(driveMotor4Port);
    pros::Motor driveMotor5(driveMotor5Port);
    pros::Motor driveMotor6(driveMotor6Port);
    
    int yInput = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    long deltaTime = pros::millis() - drivePrevTime;
    int deltaY = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - prevMotorVelocity;

    if((double)deltaY/deltaTime > maxYChange){
        yInput = prevMotorVelocity + maxYChange;
    }

    if((double)deltaY/deltaTime < maxYChangeNeg){
        yInput = prevMotorVelocity - maxYChangeNeg;
    }

    driveMotor1.move(((double)1/127)*((Simpler::sign(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    driveMotor2.move(((double)1/127)*((Simpler::sign(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    driveMotor3.move(((double)1/127)*((Simpler::sign(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput - driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    driveMotor4.move(((double)1/127)*((Simpler::sign(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    driveMotor5.move(((double)1/127)*((Simpler::sign(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));
    driveMotor6.move(((double)1/127)*((Simpler::sign(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0)?(-pow(yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2)):pow((yInput + driveController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), 2)));

    drivePrevTime = pros::millis();
    prevMotorVelocity = driveController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
}
void HDrive::moveVelocity(int yVelocity, int heading){
    // pros::Motor fLMotor(fLMotorPort);
    // pros::Motor fRMotor(fRMotorPort);
    // pros::Motor bLMotor(bLMotorPort);
    // pros::Motor bRMotor(bRMotorPort);

    // fRMotor.move(yVelocity - heading);
    // bRMotor.move(yVelocity - heading);
    // fLMotor.move(yVelocity + heading);
    // bLMotor.move(yVelocity + heading);

}
void HDrive::faceHeading(int heading, int maxErrParam, long exitTimer, bool useInertial){
    // pros::Motor fLMotor(fLMotorPort);
    // pros::Motor fRMotor(fRMotorPort);
    // pros::Motor bLMotor(bLMotorPort);
    // pros::Motor bRMotor(bRMotorPort);

    // pros::Imu inertial(inertialPort);
    // PIDController<double> headPid(true);

    // const double Kp = 190; //155 works but oscillates a bit too
    // const double Ki = 0;
    // const double Kd = 0;
    // const double windupUpperLimit = 5;

    // long deltaTime = 0;
    // long prevTime = 0;
    // long startTime = -1;
    // double error = 0;
    
    // const double maxError = maxErrParam;

    // double angleFromSet = 0; //Difference between current heading and desired heading from 0-360
    // double totalVoltage = 0;


    // while(true){
    //     if(gpsSystem->getHeading() == INFINITY && !useInertial){
    //         return;
    //     }

    //     angleFromSet = !useInertial?((((int)gpsSystem->getHeading() - heading) + 360) % 360):((((int)Simpler::degreeToStdPos(inertial.get_heading()) - heading) + 360) % 360);

    //     if(angleFromSet > 180){
    //         error = -(180 - (angleFromSet - 180));
    //     } else {
    //         error = angleFromSet;
    //     }
    //     headPid.setError(error);

    //     totalVoltage = headPid.calculateOutput(Kp, Ki, Kd, windupUpperLimit, 0, 0);
        
    //     fLMotor.move_voltage(totalVoltage);
    //     fRMotor.move_voltage(-totalVoltage);
    //     bLMotor.move_voltage(totalVoltage);
    //     bRMotor.move_voltage(-totalVoltage);

    //     if(startTime == -1){
    //         startTime = pros::millis();
    //         printf("started exittime\n");
    //     }

    //     if((pros::millis() > startTime+exitTimer && exitTimer != -1) && startTime != -1){
    //         stopAllDrive();
    //         return;
    //     }

    //     if(Simpler::abs(error) <= maxError){
    //         deltaTime = pros::millis() - prevTime;
    //         if(deltaTime > 250){
    //             stopAllDrive();
    //             return;
    //         }
    //     }
    //     else {
    //         prevTime = pros::millis();
    //     }
    // }

}
void HDrive::fieldCentricSteeringControl(pros::Controller driveController, int storedPercent, int direction){

}
void HDrive::goToPos(int x, int y, int maxErrParam, int errTimerEnableThreshold, long exitTimer){
//     pros::Motor fLMotor(fLMotorPort);
//     pros::Motor fRMotor(fRMotorPort);
//     pros::Motor bLMotor(bLMotorPort);
//     pros::Motor bRMotor(bRMotorPort);
//     // pros::Imu inertial(inertialPort);
//     PIDController<double> gtPid(true);
//     PIDController<double> angPid(true);

//     const double l_Kp = 46; //48 is mid
//     const double l_Ki = 0.04; //0.052 and 0.047 is pretty good
//     const double l_Kd = 0;

//     const double a_Kp = 147;
//     const double a_Ki = 0; //was 10
//     const double a_windupUpperLimit = 0; //was inf
//     const double maxErr = maxErrParam;
//     const double windupUpperLimit = 48;
    
//     long deltaTime = 0;
//     long prevTime = 0;
//     long startTime = -1;
//     double linError = 0;
//     double targetAng = 0;
//     double angError = 0;
//     double heading = gpsSystem->getHeading();
//     double reverseHeading = Simpler::coterminalToStdPos(gpsSystem->getHeading() + 180);
//     double prevLinError = 0;
//     int linErrSign = 1;

//     double currentX = 0;
//     double currentY = 0;
    

//     while(true){
//         currentX = gpsSystem->getPositions().x * 1000;
//         currentY = gpsSystem->getPositions().y * 1000;

//         if(gpsSystem->getHeading() == INFINITY){
//             return;
//         }

//         heading = gpsSystem->getHeading();
//         reverseHeading = Simpler::coterminalToStdPos(gpsSystem->getHeading() + 180);

//         linError = Formula::twoCoordDistance(currentX, currentY, x, y);
//         gtPid.setError(linError*linErrSign);

//         targetAng = atan2(y - currentY, x - currentX);

//         if(targetAng < 0){
//             targetAng+=(2*M_PI);
//         }

//         targetAng = targetAng * (180/M_PI);

//         angError = ((int)(heading - targetAng) + 360) % 360; //might have to use degreetostdpos here for targetAng


//         if(angError > 180){
//             angError = -(180 - (angError - 180));
//         }

//         double regAngErr = angError;

//         angError = ((int)(reverseHeading - targetAng) + 360) % 360;

//         if(angError > 180){
//             angError = -(180 - (angError - 180));
//         }

//         if(regAngErr < angError){
//             angError = regAngErr;
//         }
//         else{
//             //reverse the direction of linear movement here
//         }




        
//         if(pros::millis() % 1000 == 0){
//             // printf("linErr: %f\n", linError);
//             printf("angErr: %f\n", angError);

//         }
        

//         angPid.setError(angError);

//         double linearAppliedVoltage = gtPid.calculateOutput(l_Kp, l_Ki, l_Kd, windupUpperLimit, 0, 0);
//         double angularAppliedVoltage = angPid.calculateOutput(a_Kp, a_Ki, 0, a_windupUpperLimit, 0, 0);

//         // linearAppliedVoltage = 0;

//         fLMotor.move_voltage((-angularAppliedVoltage));
//         fRMotor.move_voltage((angularAppliedVoltage));
//         bLMotor.move_voltage((-angularAppliedVoltage));
//         bRMotor.move_voltage((angularAppliedVoltage));

//         prevLinError = linError;

//         if(prevLinError > linError){
//             linErrSign = -linErrSign;
//         }

//         if(((Simpler::abs(linError) <= errTimerEnableThreshold) && startTime == -1) || (errTimerEnableThreshold == INFINITY && startTime == -1)){
//             startTime = pros::millis();
//             printf("started exittime\n");
//         }
//         if((pros::millis() > startTime+exitTimer && exitTimer != -1) && startTime != -1){
//             stopAllDrive();
//             return;
//         }

//         if (Simpler::abs(linError) <= maxErr){
//             deltaTime = pros::millis() - prevTime;
//             if (deltaTime > 250){
//                 stopAllDrive();
//                 return;
//             }
//         } else {
//             prevTime = pros::millis();

//         } 
//   }

}

void HDrive::moveDistance(int d, int maxErrParam){
    // pros::Motor fLMotor(fLMotorPort);
    // pros::Motor fRMotor(fRMotorPort);
    // pros::Motor bLMotor(bLMotorPort);
    // pros::Motor bRMotor(bRMotorPort);

    // pros::Imu inertial(inertialPort);

    // const double l_Kp = 28; //48 is mid
    // const double l_Ki = 0; //0.04 and 0.047 is pretty good
    // const double l_Kd = 0;
    // const double l_windupUpperLimit = 5;

    // const double a_Kp = 190;
    // const double a_Ki = 0.025; //0.01
    // const double a_windupUpperLimit = 5;

    // PIDController<double> headPid(true);
    // PIDController<double> linearPid(false);

    // const double lockHeading = inertial.get_heading();

    // double angError = 0; 

    // long deltaTime = 0;
    // long prevTime = 0;

    // fLMotor.tare_position();
    // fRMotor.tare_position();
    // bLMotor.tare_position();
    // bRMotor.tare_position();

    // fRMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    // fLMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    // bRMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    // bLMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

    // while(true){
    //     if(inertial.get_heading() == INFINITY){
    //         return;
    //     }

    //     double angleFromSet = lockHeading - inertial.get_heading();

    //     double averageDistance = (((fLMotor.get_position()+fRMotor.get_position()+bRMotor.get_position()+bLMotor.get_position())/4)*gearRatio*M_PI*(wheelDiameter*25.4));

    //     if(angleFromSet > 180 || angleFromSet < -180){
    //         angError = -1*Simpler::sign(angleFromSet)*(360 - Simpler::abs(angleFromSet));
    //     } else {
    //         angError = angleFromSet;
    //     }

    //     headPid.setError(angError);

    //     double angVoltage = headPid.calculateOutput(a_Kp, a_Ki, 0, a_windupUpperLimit, 0, 0);
    //     double linearVoltage = linearPid.calculateOutput(l_Kp, l_Ki, l_Kd, l_windupUpperLimit, d, averageDistance);
        
    //     fLMotor.move_voltage(linearVoltage+angVoltage);
    //     fRMotor.move_voltage(linearVoltage-angVoltage);
    //     bLMotor.move_voltage(linearVoltage+angVoltage);
    //     bRMotor.move_voltage(linearVoltage-angVoltage);
        

    //     if(Simpler::abs(linearPid.getError()) <= maxErrParam){
    //         deltaTime = pros::millis() - prevTime;
    //         if(deltaTime > 250){
    //             stopAllDrive();
    //             return;
    //         }
    //     }
    //     else {
    //         prevTime = pros::millis();
    //     }
    // }

}


    


