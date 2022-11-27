#include "drivetrain.h"
#include "main.h"
#include "mathlib.h"
#include "cmath"
#include "drivetrain.h"
#include "pros/gps.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include "globals.hpp"
#include "controllers.hxx"

void autoAim(bool isBlueGoal, Drivetrain train);
void idleLauncher();
void moveLauncher(int flywheelRPM);