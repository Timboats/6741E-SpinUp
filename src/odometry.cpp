#include "odometry.h"
#include "pros/rtos.hpp"


Odometry::Odometry(unsigned int leftPort, unsigned int rightPort, unsigned int backPort){
    lp = leftPort;
    rp = rightPort;
    bp = backPort;

    pros::Rotation ltw(lp, true); //move reverse properties to init in main
    pros::Rotation rtw(rp, true);
    pros::Rotation btw(bp);

    ltw.set_position(0);
    rtw.set_position(0);
    btw.set_position(0);


    pros::Task odomLoop ([this]{while(true){trackingLoop();}}, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tracking wheel update");

}
void Odometry::trackingLoop(){
    pros::Rotation ltw(lp);
    pros::Rotation rtw(rp);
    pros::Rotation btw(bp);

    double Lpos = ((double)ltw.get_position()/100)*(M_PI/180); //2 is the diameter of the wheel make that a var later tho and make a method for centideg to rad
    double Rpos = ((double)rtw.get_position()/100)*(M_PI/180);

    double deltaLpos = Lpos - prevLpos;
    double deltaRpos = Rpos - prevRpos;

    prevLpos = Lpos;
    prevRpos = Rpos;

    deltaL = 2 * (deltaLpos);
    deltaR = 2 * (deltaRpos);

    currentHead += ((deltaL-deltaR)/(6.5+6.5)); // 6.5 tracking wheel distance from center

    pros::Task::delay(10);
}

double Odometry::getHeading(){
    return currentHead;
}