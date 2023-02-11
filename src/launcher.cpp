#include "launcher.h"
#include "controllers.hxx"

TBHController<double> leftFlywheelController(8000); //300
TBHController<double> rightFlywheelController(8000); //300

pros::Controller con = pros::Controller(pros::E_CONTROLLER_MASTER);
void moveLauncher(int flywheelRPM){
    pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
    pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);

    launcherMotorLeft.move_voltage(leftFlywheelController.calculateOutput(0.453, flywheelRPM, launcherMotorLeft.get_actual_velocity())); //0.352
    launcherMotorRight.move_voltage(rightFlywheelController.calculateOutput(0.59, (double)flywheelRPM*LAUNCHERMOTORRATIO, launcherMotorRight.get_actual_velocity()));
    // printf("Error R: %f\n", ((double)flywheelRPM*LAUNCHERMOTORRATIO) - launcherMotorRight.get_actual_velocity());
    // printf("Error L: %f\n", ((double)flywheelRPM) - launcherMotorLeft.get_actual_velocity());


}
void flywheelTask(void* rpm){
    while(true){
        moveLauncher((int)rpm);
        pros::Task::delay(50);
    }
    
}
float runFlightSim(float desiredDisplacement, float desiredHeight){
    printf("disc flight sim start \n");
    
    long double RHO = 1.23 * pow(10, -9); //in kg/mm/mm/mm
    //eventually this needs to change based on ambient air temp, humidity, & elevation
    float CL0 = 0;
    //the coefficient of lift independent of the angle of attack
    float CLA = 2;
    //the coefficient of lift dependent on the angle of attack
    float CD0 = 0.2;
    //the coefficient of drag independent of the angle of attack
    float CDA = 2.5;
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

    printf("z value after termination: %f, d value after termination: %f, desired z value: %f, desired d value: %f,  velocity after termination: %f \n", curZ, curDisplacement, desiredHeight, desiredDisplacement, curVelocity);
    
    return curVelocity;
}



float findRequiredRPM(float goalXPos, float goalYPos, float goalZPos, float robotXPos, float robotYPos){
    printf("findRequiredRPM start \n");

    float displacementFromGoal = ( Simpler::abs((Formula::twoCoordDistance(goalXPos, goalYPos, robotXPos, robotYPos))));
    printf("displacement from goal: %f \n", displacementFromGoal);

    float requiredVelocity = (runFlightSim(displacementFromGoal, goalZPos));
    printf("disc flight sim end \n");  

    return (requiredVelocity/VELOCITYTORPMCONST); //input is in mm/sec, output is in RPM 

}

void autoAim(bool isBlueGoal, Drivetrain train){
    con.clear();
    pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT, pros::E_MOTOR_GEARSET_06, true);
    pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT, pros::E_MOTOR_GEARSET_06);
    pros::Gps gps1(GPS1PORT);
    
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
    printf("findRequiredRPM complete");

    float angleAdjustment = 0; //a positive value aims the robot more to the left, a negative value more to the right.
    float angleFromGoal = ((int)(((std::atan2(goalYPos - robotYPos, goalXPos - robotXPos)) * (180/M_PI)) + 360) % 360) + angleAdjustment; //in degrees & simplified to 0 to 360
    
    
    train.faceHeading(angleFromGoal);
    printf("position facing complete \n");

    

    //this is for debugging feel free to remove it.
    float robotHeading = gps1.get_heading();
    printf("%f, %f, %f \n", targetRPM, angleFromGoal, robotHeading);
    /////////////////////////////

    launcherMotorRight.move_velocity(targetRPM);
    launcherMotorLeft.move_velocity(targetRPM * LAUNCHERMOTORRATIO);
    printf("motor vel set complete \n");
    pros::delay(3000);
    pros::ADIDigitalOut indexer(1, HIGH);
    pros::delay(1000);
    pros::ADIDigitalOut i(1, LOW);
    pros::delay(2000);



    launcherMotorRight.move_velocity(0);
    launcherMotorLeft.move_velocity(0);


    

    //insert launch code when launching system is complete
}
float calcIdleSpeed(float temp){
    return 9000*(1/(1+exp(0.05*(temp - 50))));
}
void idleLauncher(){
    pros::Motor launcherMotorLeft(LAUNCHERMOTORLEFTPORT);
    pros::Motor launcherMotorRight(LAUNCHERMOTORRIGHTPORT);

    launcherMotorLeft.move_voltage(calcIdleSpeed(launcherMotorLeft.get_temperature()));
    launcherMotorRight.move_voltage(calcIdleSpeed(launcherMotorRight.get_temperature()));
}