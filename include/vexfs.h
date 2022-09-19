#pragma once
#include <cstdio>
#include <cwchar>
#include <sys/unistd.h>
struct settings{
    float faceHeading_kp = 0;
    float faceHeading_ki = 0;
    float faceHeading_kd = 0;

    float goToPos_kp = 0;
    float goToPos_ki = 0;
    float goToPos_kd = 0;

};

settings getSettings(); //gets the settings stored in the settings files and returns a struct with all of its values
void writeSettings(settings tempSettings);
void fileSysInit(); //Creates all the files with default settings needed for the program to run properly