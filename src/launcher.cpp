#include "launcher.h"
#include "main.h"
#include "mathlib.h"
#include "cmath"
#include "drivetrain.h"
#include "pros/gps.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <cstddef>
#include <cstdio>


#define GRAVITY -9810 //in mm/s/s
#define DISCMASS 0.065 //in kg
#define DISCAREA 15000 //in mm^2
#define LAUNCHERHEIGHT 300 //in mm
#define LAUNCHERMOTORRATIO 3/4 
#define VELOCITYTORPMCONST 24.13 //divide to convert from mm/sec to RPM
#define launcherMotorLeftPort 9 //POV from entrance/intake of launcher
#define launcherMotorRightPort 2
#define GPS1PORT 15
pros::Controller con = pros::Controller(pros::E_CONTROLLER_MASTER);

float runFlightSim(float desiredDisplacement, float desiredHeight){
    
    long double RHO = 1.23 * pow(10, -9); //in kg/mm/mm/mm
    //eventually this needs to change based on ambient air temp, humidity, & elevation
    float CL0 = 0.1;
    //the coefficient of lift independent of the angle of attack
    float CLA = 1.4;
    //the coefficient of lift dependent on the angle of attack
    float CD0 = 0.08;
    //the coefficient of drag independent of the angle of attack
    float CDA = 2.72;
    //the coefficient of drag dependent on the angle of attack
    float ALPHA0 = 0; //The angle of attack at which lift in 0 & drag is at a minimum
    float maxError = 1; //in mm
    float Kp = 1.8; //A constant value
    float initDisplacement = 0; //in mm
    float zInit = LAUNCHERHEIGHT; //in mm
    float initVelocity = 1000; //in mm/sec
    float curDisplacement = initDisplacement; //in mm
    float curZ = zInit; //in mm
    float curVelocity = initVelocity; //in mm/sec
    float initVelocityAngle = 40; //in degrees
    float curVelocityD = curVelocity*cos(initVelocityAngle*(M_PI/180)); //in mm/sec
    float curVelocityZ = curVelocity*sin(initVelocityAngle*(M_PI/180)); //in mm/sec
    float deltaCurVelocityD = 0; //in mm/sec || D stands for displacement
    float deltaCurVelocityZ = 0; //in mm/sec
    float discAngle = 40; //in degrees
    float ALPHA = 0; //in degrees
    float curTime = 0; //in seconds
    float deltaTime = 0.0001; //in seconds
    float CL = 0; //variable for the lift coefficient
    float CD = 0; //variable for the drag coefficient
    int iterations = 0;
    int curIterations = 0;
    
    
    while(Simpler::abs(curZ - desiredHeight) > maxError){
        
        curDisplacement = initDisplacement;
        curZ = zInit;
        curTime = 0;

        
        while(curDisplacement < desiredDisplacement){
            
            ALPHA = discAngle - ((atan2(curVelocityZ, curVelocityD)) * (180/M_PI));

            CL = CL0 + CLA*ALPHA*(M_PI/180);
            CD = CD0 + CDA*pow((ALPHA - ALPHA0) * (M_PI/180), 2);

            deltaCurVelocityD = -RHO*pow(curVelocityD, 2)*DISCAREA*CD*deltaTime;
            deltaCurVelocityZ = (RHO*pow(curVelocityD, 2)*DISCAREA*CL/2/DISCMASS + GRAVITY)*deltaTime;

            curVelocityD = curVelocityD + deltaCurVelocityD;
            curVelocityZ = curVelocityZ + deltaCurVelocityZ;

            curDisplacement = curDisplacement + curVelocityD*deltaTime;
            curZ = curZ + curVelocityZ*deltaTime;

            curTime = curTime + deltaTime;

        }
       

        curVelocity = curVelocity - Kp*(curZ - desiredHeight);
        

        curVelocityD = curVelocity*cos(initVelocityAngle*(M_PI/180)); 
        curVelocityZ = curVelocity*sin(initVelocityAngle*(M_PI/180));

        iterations = iterations + 1; 
    }

    printf("iterations after completion: %d \n", iterations);

    printf("z value after termination: %f, x value after termination: %f, desired z value: %f, desired x value: %f,  velocity after termination: %f \n", curZ, curDisplacement, desiredHeight, desiredDisplacement, curVelocity);
    
    return curVelocity;
}



float findRequiredRPM(float goalXPos, float goalYPos, float goalZPos, float robotXPos, float robotYPos){
    printf("findRequiredRPM start \n");
    float displacementFromGoal = ( Simpler::abs((Formula::twoCoordDistance(goalXPos, goalYPos, robotXPos, robotYPos))));
    printf("displacement from goal: %f \n", displacementFromGoal);

    float requiredVelocity = (runFlightSim(displacementFromGoal, goalZPos));
    printf("flight sim end \n");  

    //con.print(0, 0, "flywheel RPM: %f \n", (requiredVelocity/VELOCITYTORPMCONST));
    return (requiredVelocity/VELOCITYTORPMCONST); //input is in mm/sec, output is in RPM 

}

void autoAim(bool isBlueGoal, Drivetrain train){
    con.clear();
    pros::Motor launcherMotorLeft(launcherMotorLeftPort, pros::E_MOTOR_GEARSET_06, true);
    pros::Motor launcherMotorRight(launcherMotorRightPort, pros::E_MOTOR_GEARSET_06);
    pros::Gps gps1(GPS1PORT);
    
    //printf("before \n");
    //runFlightSim(1000, 0);
    //printf("after \n");
    
    //for the purposes of testing, until we get the intertial sensor position tracking to be functional
    float robotXPos = gps1.get_status().x * 1000;
    float robotYPos = gps1.get_status().y * 1000;
    
    printf("robot pos grab Complete \n");
    printf("robot x pos: %f \n", robotXPos);
    printf("robot y pos: %f \n", robotYPos);
    ////////////////////////////////////////////////////////

    float goalZPos = 762;
    float goalXPos = 0;
    float goalYPos = 0;

    if(isBlueGoal == 1){
        goalXPos = -1332;
        goalYPos = -1332;
    } else{
        goalXPos = 1332;
        goalYPos = 1332;
    }

    float targetRPM = findRequiredRPM(goalXPos, goalYPos, goalZPos, robotXPos, robotYPos);
    printf("vel to RPM complete \n");

    launcherMotorRight.move_velocity(targetRPM);
    launcherMotorLeft.move_velocity(targetRPM * LAUNCHERMOTORRATIO);
    printf("motor vel set complete \n");

    float angleFromGoal = (int)(((std::atan2(goalYPos - robotYPos, goalXPos - robotXPos)) * (180/M_PI)) + 360) % 360; //in degrees & simplified to 0 to 360
    //con.print(1, 0, "des heading: %f \n", angleFromGoal);
    
    train.faceHeading(angleFromGoal);
    printf("position facing complete \n");

    //this is for debugging feel free to remove it.
    float robotHeading = gps1.get_heading();
    printf("%f, %f, %f \n", targetRPM, angleFromGoal, robotHeading);
    /////////////////////////////

    while(true){
        //printf("p \n");
    } //Just to stop the code from reutrning for debugging

    //insert launch code when launching system is complete
}