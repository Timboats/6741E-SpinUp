#pragma once
#include <cstdio>
#include <cwchar>
#include <sys/unistd.h>
struct settings{
    float faceHeading_kp;
    float faceHeading_ki;
    float faceHeading_kd;

    float goToPos_kp;
    float goToPos_ki;
    float goToPos_kd;

};

settings getSettings(); //gets the settings stored in the settings files and returns a struct with all of its values
void fileSysInit(); //Creates all the files with default settings needed for the program to run properly