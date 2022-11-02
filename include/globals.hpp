#pragma once
#include "main.h"
// extern pros::Controller controller;

//Device ports
#define NORTHMOTORPORT 10 
#define SOUTHMOTORPORT 19 
#define EASTMOTORPORT 9 
#define WESTMOTORPORT 20 
#define GPS1PORT 18
#define LAUNCHERMOTORLEFTPORT 9
#define LAUNCHERMOTORRIGHTPORT 2
#define INERTIALSENSORPORT 8
#define ROLLERPORT 17

#define GPSOFFSETFROMFRONT 90
#define SETTINGSPATH "/usd/code_settings.json"
#define _USE_MATH_DEFINES

//Real world and object constants
#define GRAVITY -9810 //in mm/s/s
#define DISCMASS 0.065 //in kg
#define DISCAREA 15383 //in mm^2
#define LAUNCHERHEIGHT 300 //in mm
#define LAUNCHERMOTORRATIO 3/4 
#define VELOCITYTORPMCONST 15 //divide to convert from mm/sec to RPM
