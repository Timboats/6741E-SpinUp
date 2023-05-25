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
#include "globals.hpp"
#include "odometry.h"

long matchStartTime = 0;
int driveDirection = 1;

GpsWrapper gps1(GPS1PORT, 0.0921, 0.159);
GpsWrapper gps2(GPS2PORT, -0.1, -0.2);

GpsWrapper* gps1Pointer = nullptr;
GpsWrapper* gps2Pointer = nullptr;

DualGps gpsSys(gps1Pointer, gps2Pointer, 0.1);
DualGps* gpsSysPtr = nullptr;

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
HDrive train(4, 1, DRIVEMOTOR1, DRIVEMOTOR2, DRIVEMOTOR3, DRIVEMOTOR4, DRIVEMOTOR5,DRIVEMOTOR6, INERTIALSENSORPORT, gpsSysPtr);



void initialize() {
  // master.clear();

  pros::Motor driveMotor1(DRIVEMOTOR1, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor driveMotor2(DRIVEMOTOR2, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor driveMotor3(DRIVEMOTOR3, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor driveMotor4(DRIVEMOTOR4, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor driveMotor5(DRIVEMOTOR5, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor driveMotor6(DRIVEMOTOR6, pros::E_MOTOR_GEARSET_06, true);

  gps1 = GpsWrapper(GPS1PORT, 0.0921, 0.159, GPS1OFFSETFROMFRONT);
  gps2 = GpsWrapper(GPS2PORT, 0.1, 0.1, GPS2OFFSETFROMFRONT);

  gps1Pointer = &gps1;
  gps2Pointer = &gps2;

  gpsSys = DualGps(gps1Pointer, gps2Pointer, 0.1);
  gpsSysPtr = &gpsSys;

  pros::Imu Inertial(INERTIALSENSORPORT);

  bool isInertInit = Inertial.reset();

  while (Inertial.is_calibrating() && pros::millis() < 3000) {
  }

  Inertial.set_heading(0); //was 90

  train = HDrive(4, 1, DRIVEMOTOR1, DRIVEMOTOR2, DRIVEMOTOR3, DRIVEMOTOR4, DRIVEMOTOR5,DRIVEMOTOR6, INERTIALSENSORPORT, gpsSysPtr);
}


void controllerButtonCalls(){

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == true){
    driveDirection = -driveDirection;
  }
  
  
}


void disabled() {}


void competition_initialize() {}



void autonomous() {
  
}

void opcontrol() {
  // train.goToPos(0, 0)
    // train.goToPos(0, 0);
    // coord c = Formula::findCircleIntersect(-5, 0, 0, 5, 2);
    // printf("Intersect at x: %f, y: %f\n", c.x, c.y);
    Odometry track(LEFTROTATIONPORT, RIGHTROTATIONPORT, BACKROTATIONPORT);
    pros::Imu Inertial(INERTIALSENSORPORT);

  



	while (true) {
    printf("inert: %f, track: %f\n", Inertial.get_heading(), track.getHeading()*(180/M_PI));
    // mainloopControllerUpdater();
    // if(isIdle){
    //   idleLauncher();
    // }
    // else{
    //   moveLauncher(launcherRpmOptions[currentRpmIndex]);
    //   master.rumble(rumblePattern);
    // }

    // controllerButtonCalls();
    
    // train.driverCentricSteeringControl(master, 127, 0.5);
    

    

    //0 on the field is 90 on a unit circle

		pros::delay(20);
	}
  
}
