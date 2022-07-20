#include "main.h"
#include "drivetrain.h"
#include "mathlib.h"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <string>

#define NORTHMOTORPORT 2
#define SOUTHMOTORPORT 10
#define EASTMOTORPORT 1
#define WESTMOTORPORT 9
#define GPS1PORT 16

bool buttonAState = 0;
pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
Drivetrain train(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, GPS1PORT);

/**
 * Things that might break
 * Controller axis returning values from -127 to 127 instead of -100 to 100 
 * on press button callbacks
 */

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
  return;
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();

  pros::Motor EastMotorInit(EASTMOTORPORT, pros::E_MOTOR_GEARSET_18);
  pros::Motor NorthMotorInit(NORTHMOTORPORT, pros::E_MOTOR_GEARSET_18);
  pros::Motor SouthMotorInit(SOUTHMOTORPORT, pros::E_MOTOR_GEARSET_18);
  pros::Motor WestMotorInit(WESTMOTORPORT, pros::E_MOTOR_GEARSET_18);
  pros::Gps GpsPrimaryInit(GPS1PORT, 0.00, -0.0127);

  train = Drivetrain(3.25, 1, NORTHMOTORPORT, SOUTHMOTORPORT, EASTMOTORPORT, WESTMOTORPORT, 45, 225, 135, 315, GPS1PORT);

	pros::lcd::set_text(1, "FREDDYYYY!!!");
  // std::cout << NorthMotor << std::endl;

	// pros::lcd::register_btn1_cb(on_center_button);
	//Controller1.ButtonA.pressed(toggleButtonA);

}

int errorTTP = 0;
int prevErrorTTP = 0;
//There is no loop inside the turnTowardsPoint function
//So, these variables needed to be global
//They have TTP at the end to indicate this (Turn Towards Point)

float turnTowardsPoint(int x, int y){
  pros::Gps GpsPrimary(GPS1PORT);
  float motorPercentage = 0;
  const float Kp = 1;
  const float Ki = 0.01;
  const float Kd = 0.3;
  const int windupUpperLimit = 15;
  float integral = 0;
  int angleFromDesired = 0;
  int desiredAngle = 0;

  desiredAngle = atan2(y - (GpsPrimary.get_status().y * 1000), x - (GpsPrimary.get_status().x * 1000)) * (180/M_PI); //in mm prolly gonna break

  angleFromDesired = (((desiredAngle - Simpler::degreeToStdPos(GpsPrimary.get_heading())) + 360) % 360);
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



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  pros::Gps GpsPrimary(GPS1PORT);


  // train.goToPos(0, 0, master);
  testGoToMethod();
  
  // master.set_text(1, 1, std::to_string(GpsPrimary.get_heading()));
  // pros::delay(100);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  autonomous();
	int storedPercentage = 0;

	while (true) {
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == true){
      toggleButtonA();
    }

		if (buttonAState == 1){
			storedPercentage = turnTowardsPoint(0, 0);
		} 
		else {
			storedPercentage = 0;
		}
    //This will allow the robot to turn to face a goal while still being able to be driven around

    train.steeringControl(master, storedPercentage);
    //This allows for driver control. By modifying the value outputted by the control stick, the movement of the robot is relative to the field, rather than the heading.
		pros::delay(20);
	}
}
