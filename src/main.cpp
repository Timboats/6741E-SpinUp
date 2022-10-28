#include "main.h"
#include "drivetrain.h"
#include "mathlib.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include "braingui.h"
#include "vexfs.h"
#include "launcher.h"

#define NORTHMOTORPORT 20 // old 10
#define SOUTHMOTORPORT 1 //old 11
#define EASTMOTORPORT 11 //old 20
#define WESTMOTORPORT 10 //old 1
#define GPS1PORT 15
#define LAUNCHERMOTORLEFTPORT 9
#define LAUNCHERMOTORRIGHTPORT 2
#define INERTIALSENSORPORT 8
#define GPSOFFSETFROMFRONT 90
#define ROLLERPORT 17



int driveDirection = 1;
int rollerVoltage = 0;
bool indexState = LOW;
bool isIdle = true;
int launcherRpmOptions[3] = {350, 500, 600};
unsigned int currentRpmIndex = 0;
bool isGpsAvailable = false;



pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
Drivetrain train(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 315, 135, INERTIALSENSORPORT, GPS1PORT);

void variableFsUpdate(){
  settings storedSettings = getSettings();
  if(storedSettings.isOnBlueSide){
    driveDirection = -1;
  } 
  if(!storedSettings.isOnBlueSide){
    driveDirection = 1;
  }
  if(storedSettings.isGpsAvaiable){
    isGpsAvailable = true;
  }
  if(!storedSettings.isGpsAvaiable){
    isGpsAvailable = false;
  }

}
void initialize() {
  fileSysInit();
  lvglInitEx();
  variableFsUpdate();
  

  pros::Motor EastMotorInit(EASTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor NorthMotorInit(NORTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor SouthMotorInit(SOUTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor WestMotorInit(WESTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor RollerMotorInit(ROLLERPORT, pros::E_MOTOR_GEARSET_36);
  pros::Gps GpsPrimaryInit(GPS1PORT, 0.00, 0.175);
  pros::Imu Inertial(INERTIALSENSORPORT);
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT, pros::E_MOTOR_GEARSET_06);
  pros::ADIDigitalOut indexer(1, LOW);

  NorthMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  SouthMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  EastMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  WestMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


  // Inertial.reset();

  // while(Inertial.is_calibrating())
  // {
  // }

  Inertial.set_heading(Simpler::coterminalToStdPos(GpsPrimaryInit.get_heading()+GPSOFFSETFROMFRONT));

  train = Drivetrain(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, INERTIALSENSORPORT, GPS1PORT);

}

void controllerButtonCalls(){
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  pros::Motor roller(ROLLERPORT);

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == true){
    driveDirection = -driveDirection;
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) == true){
    if(rollerVoltage != 0){
      rollerVoltage = 0;
    }
    else{
      rollerVoltage = 10000;
    }
    roller.move_voltage(rollerVoltage);
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == true){
    if(indexState == HIGH){
      indexState = LOW;
    }
    else{
      indexState = HIGH;
    }
    roller.move_voltage(rollerVoltage);
    pros::ADIDigitalOut indexer(1, indexState);
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) == true){
    isIdle = false;
    launcherMotorLeft.move_velocity(launcherRpmOptions[currentRpmIndex]*0.75);
    launcherMotorRight.move_velocity(launcherRpmOptions[currentRpmIndex]);
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) == true){
    isIdle = true;
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) == true){
    isIdle = false;
    autoAim(0, train);
    isIdle = true;
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == true){
    if(currentRpmIndex != 2){
      currentRpmIndex++;
    } 
    else{
      currentRpmIndex = 0;
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) == true){
    if(currentRpmIndex != 0){
      currentRpmIndex--;
    } 
    else{
      currentRpmIndex = 2;
    }
  }
}

float errorTTP = 0;
float prevErrorTTP = 0;
//There is no loop inside the turnTowardsPoint function
//So, these variables needed to be global
//They have TTP at the end to indicate this (Turn Towards Point)

float turnTowardsPoint(int x, int y){
  pros::Gps GpsPrimary(GPS1PORT);
  float motorPercentage = 0;
  const float Kp = 100;
  const float Ki = 0;
  const float Kd = 0;
  const int windupUpperLimit = 15;
  float integral = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;

  desiredAngle = atan2(y - (GpsPrimary.get_status().y * 1000), x - (GpsPrimary.get_status().x * 1000)) * (180/M_PI); //in mm prolly gonna break

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(Simpler::coterminalToStdPos(GpsPrimary.get_heading()+90))) + 360) % 360);
  //finds the difference in the current angle and the desired angle from 0 to 360 degrees

  if (angleFromDesired > 180){
    errorTTP = -(180 - (angleFromDesired - 180));
  } else {
    errorTTP = angleFromDesired;
  }
  //translates angleFromDesired to -180 to 180 degrees

  integral = integral + errorTTP;
  //Adds up the area of the error versus time graph from a specified time until the current time

  if (errorTTP > windupUpperLimit){
    integral = 0;
  }
  //Defines the lower limit of the integral

  motorPercentage = -(Kp * errorTTP) + (Ki*integral) + (Kd*(errorTTP - prevErrorTTP));
  //calculates the desired voltage of the motors

  prevErrorTTP = errorTTP; 
  //Used to calculate integral
    
  return(motorPercentage);
  //Value is fed into motor to facilitate desired action
}
float faceHeadingValue(int heading){
  pros::Gps GpsPrimary(GPS1PORT);
  float motorPercentage = 0;
  const float Kp = 3;
  const float Ki = 0;
  const float Kd = 0;
  const int windupUpperLimit = 15;
  float integral = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;

  desiredAngle = heading; //in mm prolly gonna break

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(Simpler::coterminalToStdPos(GpsPrimary.get_heading()+90))) + 360) % 360);
  //finds the difference in the current angle and the desired angle from 0 to 360 degrees

  if (angleFromDesired > 180){
    errorTTP = -(180 - (angleFromDesired - 180));
  } else {
    errorTTP = angleFromDesired;
  }
  //translates angleFromDesired to -180 to 180 degrees

  integral = integral + errorTTP;
  //Adds up the area of the error versus time graph from a specified time until the current time

  if (errorTTP > windupUpperLimit){
    integral = 0;
  }
  //Defines the lower limit of the integral

  motorPercentage = -(Kp * errorTTP) + (Ki*integral) + (Kd*(errorTTP - prevErrorTTP));
  //calculates the desired voltage of the motors

  prevErrorTTP = errorTTP; 
  printf("mv: %f, head: %d\n",motorPercentage, Simpler::degreeToStdPos(Simpler::coterminalToStdPos(GpsPrimary.get_heading()+90)));
  //Used to calculate integral
    
  return(motorPercentage);

}

void testGoToMethod(){
  pros::Gps GpsPrimary(GPS1PORT);

  int randomAngle = 0;
  int xRandom = 0;
  int yRandom = 0; 

  srand((unsigned) pros::millis());
  randomAngle = (rand() % 360) + 1;
  train.faceHeading(randomAngle);
  //This is to prove that the goTo method works no matter the heading of the robot

  srand((unsigned) pros::millis());
  xRandom = (rand() % 1601) - 800;

  pros::delay((rand() % 3 + 1)*1000);

  srand((unsigned) pros::millis());
  yRandom = (rand() % 1601) - 800;
  //generates a set of random coordinates

  master.clear();
  std::cout << "Dx: "+std::to_string(xRandom) << std::endl;
  std::cout << "Dy: "+std::to_string(yRandom) << std::endl;


  //This prints the desired position, that was randomly generated, on the screen of the controller

  train.goToPos(xRandom, yRandom);
  //This sends the robot to a random x and y position


  std::cout << "Ax: "+std::to_string(GpsPrimary.get_status().x * 1000) << std::endl;

  std::cout << "Ay: "+std::to_string(GpsPrimary.get_status().y * 1000) << std::endl;


  //This prints where the robot actually went on the screen of the controller

  return;
}



void disabled() {}


void competition_initialize() {}

void redSideAuton(){
  pros::Motor roller(ROLLERPORT);
  
  roller.move_relative(-450, 100);
  

}
void blueSideAuton(){
  pros::Motor roller(ROLLERPORT);
  train.goToPos(1800, -1200);
  roller.move_relative(-450, 100);

}


void autonomous() {
  // redSideAuton();
  // blueSideAuton();

  pros::Motor EastMotor(EASTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor NorthMotor(NORTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor SouthMotor(SOUTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor WestMotor(WESTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor roller(ROLLERPORT);

  EastMotor.move_voltage(12000);

  WestMotor.move_voltage(-12000); //These stay
  NorthMotor.move_voltage(12000); //These stay 

  SouthMotor.move_voltage(-12000);

  train.stopAllDrive();

  pros::delay(3000);

  roller.move_relative(-450, 100);
}

void customLauncherPidTest(){
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);

  const int rpmSetPoint = 200;
  const double Kp = 350; //200 too high 175 too low //185 almost perfect or 250 for the left //around 95 for right
  const double Ki = 0; 
  const double Kd = 0;
  const int maxErr = 3;
  const int windupUpperLimit = 8;

  double integral = 0;
  double error = 0;
  double launcherVoltage = 0;
  double startTime = pros::millis();
  double prevVolt = 0;
  double prevError = 0;
  
	while (true) {
    // controllerButtonCalls();
    // train.steeringControl(master, 0, driveDirection);
    // train.moveVelocity(0, 50, faceHeadingValue(270));

    double currentRpm = launcherMotorLeft.get_actual_velocity();

    error = rpmSetPoint - currentRpm;
    

    integral = integral + error;

    if(error > windupUpperLimit){
      integral = 0;
    }

    launcherVoltage = (Kp * error) + (Ki*integral) + (Kd*(error - prevError));


    launcherMotorLeft.move(launcherVoltage+prevVolt);
    prevVolt = launcherVoltage;
    double time = pros::millis() - startTime;

    printf("%f, %f\n", time/1000, currentRpm);

    
    prevError = error;
		pros::delay(20);
	}
  
}


void opcontrol() {
  // pros::Gps GpsPrimaryInit(GPS1PORT);
  // train.moveVelocity(0, 0, turnTowardsPoint(-1350, -1350));
  // pros::delay(5000);
  // train.stopAllDrive();
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);

  // train.goToPos(0, 0);
  
	while (true) {
    if(isIdle){
      idleLauncher();
    }
    controllerButtonCalls();
    if(isGpsAvailable){
      train.fieldCentricSteeringControl(master, 0, driveDirection);
    }
    else{
      train.driverCentricSteeringControl(master);
    }
		pros::delay(20);
	}
  
}
