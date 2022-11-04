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
    GpsWrapper(unsigned int gpsDevicePort, float xOffset, float yOffset, float headingOffset);
    float getHeading();
    wrapper_position getPositions();

    
    
    

};