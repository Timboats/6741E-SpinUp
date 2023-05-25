#include "globals.hpp"
#include "mathlib.h"
#include <sys/_intsup.h>
#include "pros/rotation.hpp"

class Odometry{
    bool initOdometry();
    void trackingLoop();
    
    double currentX = 0;
    double currentY = 0;
    double currentHead = 0;

    double prevLpos = 0;
    double prevRpos = 0;

    double deltaL = 0;
    double deltaR = 0;

    unsigned int lp;
    unsigned int rp;
    unsigned int bp;
    

    public:
    Odometry(unsigned int leftPort, unsigned int rightPort, unsigned int backPort);
    coord getPosition(); //inches
    double getHeading();
    

};

