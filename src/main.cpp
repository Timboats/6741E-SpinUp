#include "main.h"
#include "H-drive.h"
#include "gps_wrapper.h"
#include "mathlib.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
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

long matchStartTime = 0;
int driveDirection = 1;
int rollerVoltage = 0;
bool indexState = LOW;
bool isIdle = true;
int launcherRpmOptions[1] = {335};
int currentRpmIndex = -1;
bool isGpsAvailable = false;
bool isOnBlue = false;

bool launcherGateVal = false;
bool endgame1 = false;
bool endgame2 = false;

char* rumblePattern = "-";
GpsWrapper gps1(GPS1PORT, 0.0921, 0.159);
GpsWrapper gps2(GPS2PORT, -0.1, -0.2);

GpsWrapper* gps1Pointer = nullptr;
GpsWrapper* gps2Pointer = nullptr;

DualGps gpsSys(gps1Pointer, gps2Pointer, 0.1);
DualGps* gpsSysPtr = nullptr;

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
HDrive train(4, ((double)84/32), L_FRONTMOTORPORT, R_FRONTMOTORPORT, L_BACKMOTORPORT, R_BACKMOTORPORT, INERTIALSENSORPORT, gpsSysPtr);


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
  // master.clear();
  
  pros::Motor FLMotorInit(L_FRONTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor FRMotorInit(R_FRONTMOTORPORT, pros::E_MOTOR_GEARSET_18, false);
  pros::Motor BLMotorInit(L_BACKMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor BRMotorInit(R_BACKMOTORPORT, pros::E_MOTOR_GEARSET_18, false);

  pros::Motor RollerMotorInit(ROLLERPORT, pros::E_MOTOR_GEARSET_36, false);
  pros::Motor IntakeMotorInit(INTAKEPORT, pros::E_MOTOR_GEARSET_36, false);

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

  FRMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  BLMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  FLMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  BRMotorInit.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  bool isInertInit = Inertial.reset();

  while (Inertial.is_calibrating() && pros::millis() < 3000) {
  }

  Inertial.set_heading(90);

  train = HDrive(4, ((double)84/32), L_FRONTMOTORPORT, R_FRONTMOTORPORT, L_BACKMOTORPORT, R_BACKMOTORPORT, INERTIALSENSORPORT, gpsSysPtr);
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
    roller.move_voltage(10000);
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) == false){
    roller.move_voltage(0);
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true){
    intake.move_voltage(12000);
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == true){
    intake.move_voltage(-12000);
    
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
      launcherGateVal = false;
      pros::ADIDigitalOut launcherGate(7);
      launcherGate.set_value(launcherGateVal);
    }
    else{
      launcherGateVal = true;
      pros::ADIDigitalOut launcherGate(7);
      launcherGate.set_value(launcherGateVal);
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
      launcherGateVal = false;
      pros::ADIDigitalOut launcherGate(7);
      launcherGate.set_value(launcherGateVal);
    }
    else{
      launcherGateVal = true;
      pros::ADIDigitalOut launcherGate(7);
      launcherGate.set_value(launcherGateVal);
    }
    
  }
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == false && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == false){
    intake.move_voltage(0);
  }
  
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == true){
    isGpsAvailable = !isGpsAvailable;
  }

  if((pros::millis() >= (matchStartTime+87000)) && pros::competition::is_connected() && !pros::competition::is_disabled()){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == true) {
    endgame1 = !endgame1;
    pros::ADIDigitalOut endgameNutDropper1(1);
    endgameNutDropper1.set_value(endgame1);
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == true) {
    endgame2 = !endgame2;
    pros::ADIDigitalOut endgameNutDropper2(8);
    endgameNutDropper2.set_value(endgame2);
    }

  }
  else if(!pros::competition::is_connected()){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == true) {
    endgame1 = !endgame1;
    pros::ADIDigitalOut endgameNutDropper1(1);
    endgameNutDropper1.set_value(endgame1);
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == true) {
    endgame2 = !endgame2;
    pros::ADIDigitalOut endgameNutDropper2(8);
    endgameNutDropper2.set_value(endgame2);
    }
  }
  
  
}
void mainloopControllerUpdater(){
  char* rpm = "";
  
  if(currentRpmIndex == -1){
    rpm = "Idle";
  } 
  if(currentRpmIndex == 0){
    rpm = "Low ";
  }
  if(currentRpmIndex == 1){
    rpm = "High";
  }

  
  master.print(0, 0, "RPM: %s", rpm);
  

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

void basicRollerGetter(){
  pros::Motor roller(ROLLERPORT);

  train.moveVelocity(-100, 0);
  pros::delay(1500);
  roller.move_relative(400, 100);
  pros::delay(250);
  train.stopAllDrive();
  train.moveVelocity(100, 0);
  pros::delay(250);
  train.stopAllDrive();
}
void leftSideAuton(){
  pros::Motor roller(ROLLERPORT);
  pros::Motor intake(INTAKEPORT);
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  int* rpm = (int*)428; //427
  pros::Task flywheel (flywheelTask, rpm, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "auton ramp flywheel");

  basicRollerGetter();

  pros::delay(150);
  train.moveVelocity(0, -55);
  pros::delay(245);
  train.stopAllDrive();



  pros::delay(150);
  train.moveVelocity(100, 35, 0);
  pros::delay(1900);
  train.moveVelocity(70, 24, 0);
  pros::delay(2100);
  train.stopAllDrive();

  pros::delay(150);

  train.moveVelocity(0, 0, -55);
  pros::delay(370); //390
  train.stopAllDrive();
  pros::delay(150);
  // train.moveVelocity(0, 0, -55);
  // pros::delay(399); //450
  // train.stopAllDrive();
  // pros::delay(150);

  train.moveVelocity(0, 100, 0);
  pros::delay(250); //330
  train.stopAllDrive();
  // train.moveVelocity(0, 50, 0);
  // pros::delay(330); //310
  // train.stopAllDrive();

  pros::delay(1000);

  

  pros::ADIDigitalOut launcherGate(7);
  launcherGate.set_value(true);
  pros::delay(1500);
  intake.move_voltage(12000);
  pros::delay(389);
  intake.move_velocity(0);

  pros::delay(2500);//time between shots
  intake.move_voltage(12000);
  pros::delay(789);
  intake.move_velocity(0);

  launcherGate.set_value(false);

}

void rightSideAuton(){

}


void autonomous() {
  /*
  //starts the flywheel
  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);
  int* rpm = (int*)390;
  pros::Task flywheel (flywheelTask, rpm, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "auton ramp flywheel");
  pros::delay(10000);
  //suspends the flywheel
  flywheel.suspend();
  launcherMotorLeft.move_velocity(0);
  launcherMotorRight.move_velocity(0);
  //resumes the flywheel
  pros::delay(2000);
  flywheel.resume();
  */

  

  // pros::Motor roller(ROLLERPORT);

  // if(!isOnBlue){
  //   train.moveVelocity(0, -100, 0);
  //   pros::delay(2000);
  //   roller.move_relative(-200, 100);
  //   pros::delay(500);
  //   train.stopAllDrive();
  //   train.moveVelocity(0, 100, 0);
  //   pros::delay(250);
  //   train.stopAllDrive();
  // }
  // if(isOnBlue){
  //   train.moveVelocity(0, -100, 0);
  //   pros::delay(2000);
  //   roller.move_relative(-800, 100);
  //   pros::delay(500);
  //   train.stopAllDrive();
  //   train.moveVelocity(0, 100, 0);
  //   pros::delay(250);
  //   train.stopAllDrive();
  // }

  //sides pov of lower goal
  leftSideAuton();
  

  


  
  // flywheel.remove(); always put this at the end to end the task 
}



void opcontrol() {
  if(pros::competition::is_connected() && !pros::competition::is_disabled()){
    matchStartTime = pros::millis();
  }
  


  pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
  pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);

	while (true) {
    // mainloopControllerUpdater();
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
