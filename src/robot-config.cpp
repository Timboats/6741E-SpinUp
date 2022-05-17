#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor EastMotor = motor(PORT1, ratio18_1, false);
motor NorthMotor = motor(PORT2, ratio18_1, false);
motor SouthMotor = motor(PORT10, ratio18_1, false);
motor WestMotor = motor(PORT9, ratio18_1, false);
gps GPS16 = gps(PORT16, 0.00, -12.70, mm, 180);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}