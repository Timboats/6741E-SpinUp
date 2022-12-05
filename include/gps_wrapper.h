#pragma once
#include "main.h"
#include "mathlib.h"


typedef struct wrapperPosition {
    double x;
    double y;
} wrapper_position;

class GpsWrapper{
    private:
    unsigned int gpsDevicePort;
    float headingOffset;
    public:
    GpsWrapper(unsigned int gpsDevicePort, float xOffset, float yOffset, float headingOffset = 0);
    float getHeading();
    double getRMS();
    wrapper_position getPositions();

    
    
    

};
class DualGps{
    private:
    GpsWrapper* gps1ptr;
    GpsWrapper* gps2ptr;
    double rmsThreshold = 0.1;

    public:
    DualGps(GpsWrapper* gps1ptr, GpsWrapper* gps2ptr, double rmsThreshold);
    double getHeading();
    wrapper_position getPositions();


};