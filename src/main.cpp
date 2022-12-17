#include "main.h"
#include "drivetrain.h"
#include "gps_wrapper.h"
#include "mathlib.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include "braingui.h"
#include "vexfs.h"
#include "launcher.h"
#include "globals.hpp"


int driveDirection = 1;
int rollerVoltage = 0;
bool indexState = LOW;
bool isIdle = true;
int launcherRpmOptions[2] = {375, 410};
int currentRpmIndex = -1;
bool isGpsAvailable = false;
bool isOnBlue = false;

bool launcherGateVal = false;
bool endgame = false;

char* rumblePattern = "-";
GpsWrapper gps1(GPS1PORT, 0.0921, 0.159);
GpsWrapper gps2(GPS2PORT, -0.1, -0.2);

GpsWrapper* gps1Pointer = nullptr;
GpsWrapper* gps2Pointer = nullptr;

DualGps gpsSys(gps1Pointer, gps2Pointer, 0.1);
DualGps* gpsSysPtr = nullptr;

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
Drivetrain train(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 315, 135, INERTIALSENSORPORT, gpsSysPtr);


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
  if(storedSettings.isOnBlueSide){
    isOnBlue = true;
  }
  if(!storedSettings.isOnBlueSide){
    isOnBlue = false;
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
  pros::Motor RollerMotorInit(ROLLERPORT, pros::E_MOTOR_GEARSET_36, false);
  pros::Motor IntakeMotorInit(INTAKEPORT, pros::E_MOTOR_GEARSET_36, true);

  gps1 = GpsWrapper(GPS1PORT, 0.0921, 0.159, GPS1OFFSETFROMFRONT);
  gps2 = GpsWrapper(GPS2PORT, 0.1, 0.1, GPS2OFFSETFROMFRONT); 

  gps1Pointer = &gps1;
  gps2Pointer = &gps2;

  gpsSys = DualGps(gps1Pointer, gps2Pointer, 0.1);
  gpsSysPtr = &gpsSys;

  pros::Imu Inertial(INERTIALSENSORPORT);
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT, pros::E_MOTOR_GEARSET_06, true);

  pros::ADIDigitalOut endgameNutDropper1(1);
  pros::ADIDigitalOut endgameNutDropper2(8);
  pros::ADIDigitalOut launcherGate(7);

  NorthMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  SouthMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  EastMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  WestMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


  // Inertial.reset();

  // while(Inertial.is_calibrating())
  // {
  // }

  // Inertial.set_heading(Simpler::coterminalToStdPos(GpsPrimaryInit.get_heading()+GPSOFFSETFROMFRONT));

  train = Drivetrain(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, INERTIALSENSORPORT, gpsSysPtr);
}

void controllerButtonCalls(){
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  pros::Motor roller(ROLLERPORT);
  pros::Motor intake(INTAKEPORT);

  
  

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == true){
    driveDirection = -driveDirection;
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == true){
    if(rollerVoltage != 0){
      rollerVoltage = 0;
    }
    else{
      rollerVoltage = 10000;
    }
    roller.move_voltage(rollerVoltage);
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true){
    intake.move_voltage(12000);
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == true){
    intake.move_voltage(-12000);
    
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) == true){
    launcherGateVal = !launcherGateVal;
    pros::ADIDigitalOut launcherGate(7);
    launcherGate.set_value(launcherGateVal);
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) == true){
    isIdle = false;
    if(currentRpmIndex != (sizeof(launcherRpmOptions)/sizeof(launcherRpmOptions[0]))-1){
      currentRpmIndex++;
      if(((sizeof(launcherRpmOptions)/sizeof(launcherRpmOptions[0]))-1) == currentRpmIndex){
        rumblePattern = ".";

      }
      else {
        rumblePattern = "-";
      }
    } 
    else{
      currentRpmIndex = -1;
    }
    
    if(currentRpmIndex == -1){
      isIdle = true;
    }
    else{
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) == true){
    isIdle = false;
    if(currentRpmIndex != -1){
      currentRpmIndex--;
      rumblePattern = "-";
    } 
    else{
      currentRpmIndex = (sizeof(launcherRpmOptions)/sizeof(launcherRpmOptions[0]))-1;
      rumblePattern = ".";
    }
    if(currentRpmIndex == -1){
      isIdle = true;
    }
    else{
      
    }
    
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == false && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == false){
    intake.move_voltage(0);
  }
  
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == true){
    isGpsAvailable = !isGpsAvailable;
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == true){
    endgame = !endgame;
    pros::ADIDigitalOut endgameNutDropper(8);
    endgameNutDropper.set_value(endgame);
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

void redLeftSideAuton(){
  pros::Motor roller(ROLLERPORT);

  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(400, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(250);
  train.stopAllDrive();
}
void blueLeftSideAuton(){
  pros::Motor roller(ROLLERPORT);

  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(400, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(250);
  train.stopAllDrive();

}
void blueRightSideAuton(){
  pros::Motor roller(ROLLERPORT);

  train.moveVelocity(0, 100, 0);
  pros::delay(400);
  train.stopAllDrive();

  train.moveVelocity(100, 0, 0);
  pros::delay(1190);
  train.stopAllDrive();

  train.moveVelocity(0, -100, 0);
  pros::delay(500);
  train.stopAllDrive();

  // train.moveVelocity(100, 0, 0);
  // pros::delay(600);
  // train.stopAllDrive();

  blueLeftSideAuton();



  // roller.move_relative(400, 100);
  // pros::delay(250);
  // train.stopAllDrive();
  // train.moveVelocity(0, 100, 0);
  // pros::delay(250);
  // train.stopAllDrive();

}

void redRightSideAuton(){
  pros::Motor roller(ROLLERPORT);
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  pros::Motor intake(INTAKEPORT);
  int velocity = 400;
  

  train.moveVelocity(0, 100, 0);
  pros::delay(400);
  train.stopAllDrive();

  train.moveVelocity(100, 0, 0);
  pros::delay(1190);
  train.stopAllDrive();

  train.moveVelocity(0, -100, 0);
  pros::delay(500);
  train.stopAllDrive();

  // train.moveVelocity(100, 0, 0);
  // pros::delay(600);
  // train.stopAllDrive();

  redLeftSideAuton();


  /*
  *high goal shot portion 
  */

  //start flywheel here
  launcherMotorLeft.move_velocity(velocity);
  launcherMotorRight.move_velocity(velocity*LAUNCHERMOTORRATIO);

  train.moveVelocity(0, 100, 0);
  pros::delay(150);
  train.stopAllDrive();

  train.moveVelocity(0, 0, 80);
  pros::delay(70); //90
  train.stopAllDrive();

  pros::delay(2500);

  pros::ADIDigitalOut launcherGate(7);
  launcherGate.set_value(HIGH);
  pros::delay(100);

  intake.move_voltage(12000);

  pros::delay(600);

  intake.move_voltage(0);

  pros::delay(6000);

  intake.move_voltage(12000);

  pros::delay(2000);

  intake.move_voltage(0);


  

  

  // train.moveVelocity(0, 0, 100);
  // pros::delay(215);
  // train.stopAllDrive();

  // train.moveVelocity(-100, -45, 0);
  // pros::delay(2500);
  // train.stopAllDrive();

  // train.moveVelocity(0, 0, -100);
  // pros::delay(100);
  // train.stopAllDrive();

  // train.moveVelocity(0, 100, 0);
  // pros::delay(150);
  // train.stopAllDrive();



  

  

  
  



  // roller.move_relative(400, 100);
  // pros::delay(250);
  // train.stopAllDrive();
  // train.moveVelocity(0, 100, 0);
  // pros::delay(250);
  // train.stopAllDrive();

}

void redLeftSideSkills(){
  pros::Motor roller(ROLLERPORT);
  pros::Motor intake(INTAKEPORT);
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  int velocity = 490;

  //roller getter mode
  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(600, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(1100);
  train.stopAllDrive();
  pros::delay(100);
  //end of roller getter mode
  //roller 2 pos
  train.goToPos(-995, 1100);
  pros::delay(100);
  train.faceHeading(270);
  pros::delay(100);

  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(600, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(1100);
  train.stopAllDrive();
  pros::delay(100);
  

  //roller 3 pos
  train.goToPos(995, -1100);
  pros::delay(100);
  train.faceHeading(90);
  pros::delay(100);

  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(600, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(1100);
  train.stopAllDrive();
  pros::delay(100);

  launcherMotorLeft.move_velocity(velocity);
  launcherMotorRight.move_velocity(velocity*LAUNCHERMOTORRATIO);

  //roller 4 pos
  train.goToPos(1100, -1175);
  train.moveVelocity(0, 0, 100);
  pros::delay(1000);
  train.stopAllDrive();
  pros::delay(200);
  train.faceHeading(180);
  pros::delay(100);

  train.moveVelocity(0, -100, 0);
  pros::delay(1500);
  roller.move_relative(600, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(0, 100, 0);
  pros::delay(1100);
  train.stopAllDrive();
  pros::delay(100);
  

  //red high goal pos
  train.goToPos(1365, 330);
  pros::delay(100);
  train.faceHeading(90);
  
  pros::delay(500);
  
  pros::ADIDigitalOut launcherGate(7);
  launcherGate.set_value(true);
  intake.move_voltage(10000);
  pros::delay(2000);
  intake.move_voltage(0);
  launcherGate.set_value(false);
  pros::delay(1750);
  intake.move_voltage(10000);
  launcherGate.set_value(true);
  pros::delay(1500);
  launcherGate.set_value(false);
  pros::delay(1000);

  launcherMotorLeft.move_velocity(0);
  launcherMotorRight.move_velocity(0);

  intake.move_voltage(-7000);

  //endgame
  train.goToPos(200, 200, 100);
  pros::delay(500);
  train.faceHeading(225);
  pros::delay(1000);
  pros::ADIDigitalOut endgameNutDropper1(1);
  pros::ADIDigitalOut endgameNutDropper2(8);
  endgameNutDropper1.set_value(true);
  pros::delay(750);
  endgameNutDropper2.set_value(true);
  pros::delay(500);

  // train.moveVelocity(100, 100, 0);
  // pros::delay(10000);
  // train.stopAllDrive();


}


void autonomous() {

  // redSideAuton();
  // blueSideAuton();

  pros::Motor roller(ROLLERPORT);
  redLeftSideSkills();
}



void opcontrol() {
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);

  train.faceHeading(270);
 
  // train.goToPos(0, 0);
  // train.goToPos(-400, -400);
  // train.goToPos(400, 400);
  // train.goToPos(-400, 400);
  // train.goToPos(400, -400);
  // train.goToPos(0, 0);

	while (true) {
    if(isIdle){
      idleLauncher();
    }
    else{
      moveLauncher(launcherRpmOptions[currentRpmIndex]);
      master.rumble(rumblePattern);
    }

    controllerButtonCalls();
    if(isGpsAvailable){
      train.fieldCentricSteeringControl(master, 0, driveDirection);
    }
    else{
      train.driverCentricSteeringControl(master);
    }

    

    //0 on the field is 90 on a unit circle

		pros::delay(20);
	}
  
}
