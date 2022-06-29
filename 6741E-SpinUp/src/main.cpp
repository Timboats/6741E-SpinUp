#include "main.h"
#include "drivetrain.h"
#include "mathlib.h"
#include "pros/rtos.hpp"
#include <string>

bool buttonAState = 0;


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor EastMotor(1, pros::E_MOTOR_GEARSET_18);
pros::Motor NorthMotor(2, pros::E_MOTOR_GEARSET_18);
pros::Motor SouthMotor(10, pros::E_MOTOR_GEARSET_18);
pros::Motor WestMotor(9, pros::E_MOTOR_GEARSET_18);
pros::Gps GpsPrimary(16, 0.00, -0.0127, 180); //if bugs check this line

Drivetrain train(3.25, 1, NorthMotor, SouthMotor, EastMotor, WestMotor, 45, 225, 135, 315, GpsPrimary);


/**
 * Things that might break
 *
 * Controller axis returning values from -127 to 127 instead of -100 to 100
 * Me not recalling the constructors of everything in on_init
 * Me using pointers in for motors and gps 
 * me using get_status().x * 1000 to get pos in mm
 * on press button callbacks
 * me using pros::millis()
 */

void toggleButtonA(){
  if (buttonAState == 0){
    buttonAState = 1;
  } else {
    buttonAState = 0;
  }
  return;
}



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	//Controller1.ButtonA.pressed(toggleButtonA);

}

int errorTTP = 0;
int prevErrorTTP = 0;
//There is no loop inside the turnTowardsPoint function
//So, these variables needed to be global
//They have TTP at the end to indicate this (Turn Towards Point)

float turnTowardsPoint(int x, int y){
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
  master.set_text(1, 1, std::to_string(xRandom));
  master.set_text(2, 1, std::to_string(yRandom));
  //This prints the desired position, that was randomly generated, on the screen of the controller

  train.goToPos(xRandom, yRandom);
  //This sends the robot to a random x and y position

  master.set_text(1, 7, std::to_string(GpsPrimary.get_status().x * 1000));

  master.set_text(2, 7, std::to_string(GpsPrimary.get_status().y * 1000));

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
	// train.goToPos(0, 0);

  testGoToMethod();
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
	int storedPercentage = 0;

	while (true) {

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
