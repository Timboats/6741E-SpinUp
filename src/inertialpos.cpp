#include "inertialpos.h"
#include "pros/imu.hpp"
#include "pros/rtos.h"

float updateXVelocity(float deltaTime, float prevXVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevXVelocity+(((int)inert.get_accel().x*9.80665)*deltaTime);
}
float updateYVelocity(float deltaTime, float prevYVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevYVelocity+((inert.get_accel().y*9.80665)*deltaTime);
}
float updateZVelocity(float deltaTime, float prevZVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevZVelocity+((inert.get_accel().z*9.80665)*deltaTime);
}

float getCurrentVelocity(){
    float currentVelocity;

    return currentVelocity;
}