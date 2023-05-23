#include "globals.hpp"
#include "mathlib.h"

class Odometry{
    bool initOdometry();
    void trackingLoop();
    coord getPosition(); //inches
    double getHeading();

};

