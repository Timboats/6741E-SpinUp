#include "gps_wrapper.h"



GpsWrapper::GpsWrapper(unsigned int gpsDevicePort, float xOffset, float yOffset, float headingOffset = 0){
    this->gpsDevicePort = gpsDevicePort;
    this->headingOffset = headingOffset;

    pros::Gps initializer(gpsDevicePort, xOffset, yOffset);
}

float GpsWrapper::getHeading(){
    //returns gps heading in degrees in standard postion
    pros::Gps gpsObj(gpsDevicePort);
    return Simpler::degreeToStdPos(Simpler::coterminalToStdPos(gpsObj.get_heading()+headingOffset));
}
wrapper_position GpsWrapper::getPositions(){
    pros::Gps gpsObj(gpsDevicePort);
    wrapper_position positions;
    positions.x = gpsObj.get_status().x;
    positions.y = gpsObj.get_status().y;
    return positions;
}
