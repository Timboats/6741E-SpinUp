#include "inertialpos.h"
#include "pros/imu.hpp"
#include "pros/rtos.h"

float updateXVelocity(float deltaTime, float prevXVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevXVelocity+(inert.get_accel().x*deltaTime);
}
float updateYVelocity(float deltaTime, float prevYVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevYVelocity+(inert.get_accel().y*deltaTime);
}
float updateZVelocity(float deltaTime, float prevZVelocity, unsigned int inertialPort){
    pros::Imu inert(inertialPort);
    
    return prevZVelocity+(inert.get_accel().z*deltaTime);
}

float getCurrentVelocity(){
    float currentVelocity;

    return currentVelocity;
}