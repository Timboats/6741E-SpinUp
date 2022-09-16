#include "launcher.h"
#include "mathlib.h"
#include "cmath"
#include <cstddef>
#include <cstdio>

#define GRAVITY -9810 //in mm/s/s
#define DISCMASS 0.065 //in kg
#define DISCAREA 15328 //in mm^2
#define LAUNCHERHEIGHT 300 //in mm

float runFlightSim(float desiredDisplacement, float desiredHeight){
    float RHO = 1.23; //in g/mm/mm/mm
    //eventually this needs to change based on ambient air temp, humidity, & elevation
    float CL0 = 0.1; //The lift coefficient indepedent of angle of attack
    float CLA = 1.4; //The lift coefficient dependent on angle of attack
    float CD0 = 0.08; //The drag coefficient indepedent of angle of attack
    float CDA = 2.72; //The drag coefficient dependent on angle of attack
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
    
    while(Simpler::abs(curZ - desiredHeight) > maxError){
        curDisplacement = initDisplacement;
        curZ = zInit;
        curTime = 0;

        while(curDisplacement < desiredDisplacement){
            ALPHA = discAngle = atan2(curVelocityZ,curVelocityD);

            CL = CL0 + CLA*ALPHA*(M_PI/180);
            CD = CD0 + CDA*pow((ALPHA - ALPHA0) * (M_PI/180), 2);

            deltaCurVelocityD = -RHO*pow(curVelocityD, 2)*DISCAREA*CD*deltaTime;
            deltaCurVelocityZ = (RHO*pow(curVelocityD, 2)*DISCAREA*CL/2/DISCMASS + GRAVITY)*deltaTime;

            curVelocityD = curVelocityD + deltaCurVelocityD;
            curVelocityZ = curVelocityZ + deltaCurVelocityZ;

            curDisplacement = curDisplacement + curVelocityD;
            curZ = curZ + curVelocityZ;

            curTime = curTime + deltaTime;
        }

        curVelocity = curVelocity - Kp*(curZ - desiredHeight);

        curVelocityD = curVelocity*cos(initVelocityAngle*(M_PI/180)); 
        curVelocityZ = curVelocity*sin(initVelocityAngle*(M_PI/180));

        iterations = iterations + 1; 
    }

    return curVelocity;
}

float findRequiredRPM(float goalXPos, float goalYPos, float goalZPos, float robotXPos, float robotYPos){
    float displacementFromGoal = Simpler::abs(Formula::twoCoordDistance(goalXPos, goalYPos, robotXPos, robotYPos));


}

void autoAim(bool isBlueGoal){
    float goalZPos = 30;

    if(isBlueGoal == 1){
        float goalXPos = -1332;
        float goalYPos = -1332;
    } else{
        float goalXPos = 1332;
        float goalYPos = 1332;
    }

    std::printf("%f\n", runFlightSim(2000, 1000));



    

}