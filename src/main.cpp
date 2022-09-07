#include "main.h"
#include "drivetrain.h"
#include "mathlib.h"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <string>
#include "braingui.h"
#include "vexfs.h"

#define NORTHMOTORPORT 10
#define SOUTHMOTORPORT 13
#define EASTMOTORPORT 20
#define WESTMOTORPORT 1
#define GPS1PORT 15
#define launcherMotorLeftPort 18 //POV from entrance/intake of launcher
#define launcherMotorRightPort 19

bool buttonAState = 0;
float leftMotorVelocity = 0;
float rightMotorVelocity = 0;

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
Drivetrain train(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 315, 135, GPS1PORT);

void controllerInput(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) == true){
    if(leftMotorVelocity != -600){
      leftMotorVelocity-=20;
      master.print(0, 0, "Left: %f", leftMotorVelocity);
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == true){
    if(leftMotorVelocity != 600){
      leftMotorVelocity+=20;
      master.print(0, 0, "Left: %f", leftMotorVelocity);
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == true){
    if(rightMotorVelocity != -600){
      rightMotorVelocity-=20;
      master.print(1, 0, "Right: %f", rightMotorVelocity);
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == true){
    if(rightMotorVelocity != 600){
      rightMotorVelocity+=20;
      master.print(1, 0, "Right: %f", rightMotorVelocity);
    }
  }
}

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
  return;
}

void initialize() {
  fileSysInit(); //Checks if all needed files exist. If not it creates them
  //TODO replace the below line with the custom init
  lvglInitEx();
  

  pros::Motor EastMotorInit(EASTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor NorthMotorInit(NORTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor SouthMotorInit(SOUTHMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor WestMotorInit(WESTMOTORPORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Gps GpsPrimaryInit(GPS1PORT, 0.00, -0.03);

  pros::Motor launcherMotorLeft(launcherMotorLeftPort, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor launcherMotorRight(launcherMotorRightPort, pros::E_MOTOR_GEARSET_06);

  master.clear();
  

  train = Drivetrain(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, GPS1PORT);


	//Controller1.ButtonA.pressed(toggleButtonA);

}



void disabled() {}


void competition_initialize() {}


void autonomous() {

}


void opcontrol() {
	while (true) {
    controllerInput();
    pros::Motor launcherMotorLeft(launcherMotorLeftPort);
    pros::Motor launcherMotorRight(launcherMotorRightPort);

    launcherMotorLeft.move_velocity(leftMotorVelocity);
    launcherMotorRight.move_velocity(rightMotorVelocity);

    // float deltaTime = 1000;
    // float prevTime = 0;

    // if(pros::millis() - prevTime > deltaTime){
      std::cout << "Right: " << launcherMotorRight.get_actual_velocity() << " " << "Left: " << launcherMotorLeft.get_actual_velocity() << std::endl;
    //   prevTime = pros::millis();
    // } 

	  pros::delay(20);
	}
}
