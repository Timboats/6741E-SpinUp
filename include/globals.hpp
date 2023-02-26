#pragma once
#include "main.h"
// extern pros::Controller controller;

//Device ports
#define R_FRONTMOTORPORT 11 
#define L_BACKMOTORPORT 10 
#define R_BACKMOTORPORT 1 
#define L_FRONTMOTORPORT 20

#define GPS1PORT 13
#define GPS2PORT 18 
#define LAUNCHERMOTORLEFTPORT 21
#define INERTIALSENSORPORT 5
#define LAUNCHERSENSORPORT 9
#define ROLLERPORT 12
#define INTAKEPORT 3

#define GPS1OFFSETFROMFRONT 0
#define GPS2OFFSETFROMFRONT 180
#define SETTINGSPATH "/usd/code_settings.json"
#define _USE_MATH_DEFINES

//Real world and object constants
#define GRAVITY -9810 //in mm/s/s
#define DISCMASS 0.065 //in kg
#define DISCAREA 15383 //in mm^2
#define LAUNCHERHEIGHT 300 //in mm
#define VELOCITYTORPMCONST 15 //divide to convert from mm/sec to RPM

#define DRIVERDURATION 105000
