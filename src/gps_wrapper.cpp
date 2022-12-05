#include "gps_wrapper.h"
#include <cmath>



GpsWrapper::GpsWrapper(unsigned int gpsDevicePort, float xOffset, float yOffset, float headingOffset){
    this->gpsDevicePort = gpsDevicePort;
    this->headingOffset = headingOffset;

    pros::Gps initializer(gpsDevicePort, xOffset, yOffset);
}

float GpsWrapper::getHeading(){
    //returns gps heading in degrees in standard postion

    //add checks for when the gps is disconnected
    pros::Gps gpsObj(gpsDevicePort);
    return Simpler::degreeToStdPos(Simpler::coterminalToStdPos(gpsObj.get_heading()+headingOffset));
}
double GpsWrapper::getRMS(){
    pros::Gps gpsObj(gpsDevicePort);
    return gpsObj.get_error();
}
wrapper_position GpsWrapper::getPositions(){
    //add checks for when the gps is disconnected
    pros::Gps gpsObj(gpsDevicePort);
    wrapper_position positions;
    positions.x = gpsObj.get_status().x;
    positions.y = gpsObj.get_status().y;
    return positions;
}

DualGps::DualGps(GpsWrapper* gps1ptr, GpsWrapper* gps2ptr){
    this->gps1ptr = gps1ptr;
    this->gps2ptr = gps2ptr;
}
double DualGps::getHeading(){
    double gps1RMS = gps1ptr->getRMS();
    double gps2RMS = gps2ptr->getRMS();
    const double rmsThreshold = 0.1;
    double returnedHeading = INFINITY;

    if(gps1RMS < rmsThreshold && gps2RMS < rmsThreshold){
        //average heading
        returnedHeading = (gps1ptr->getHeading()+gps2ptr->getHeading())/2;
    }
    else if (gps1RMS > gps2RMS){
        //lesser heading
        returnedHeading = gps2ptr->getHeading();
    }
    else{
        returnedHeading = gps1ptr->getHeading();
    }
    return returnedHeading;
}
