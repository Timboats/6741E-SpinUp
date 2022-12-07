#include "mathlib.h"
#include <cmath>


float Formula::twoCoordDistance(float x1, float y1, float x2, float y2){
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
int Simpler::degreeToStdPos(int angle){
  return(((360 - angle) + 90) % 360);    
}
float Simpler::abs(float val){
  if(val < 0){
    return val*-1;
  }
  return val;
}
int Simpler::sign(float val){
  //Returns the sign of a number/negative or positive
  if (val >= 0){
    return(1);
  }
  return(-1);
}
float Formula::findComponentVector(float magnitudeOne, float angleOne, float angleTwo){
  float magnitudeTwo = magnitudeOne * cos(angleOne - angleTwo);
  return magnitudeTwo;
}
float Simpler::coterminalToStdPos(float degree){
  if(degree > 360){
    return degree - 360;
  }
  if(degree < 0){
    return degree + 360;
  }
  return degree;
}
double Simpler::degAvgTwoAngles(double angle1, double angle2){
  //TODO method kind of works/works enough for its purpose... still needs to be fixed
  double rad1 = angle1 * (M_PI/180);
  double rad2 = angle2 * (M_PI/180);

  double x = 0.5*(cos(rad1)+cos(rad2));
  double y = 0.5*(sin(rad1)+sin(rad2));
  double arcTan = atan2(y, x);
  double finalRad;

  if(x > 0 && y > 0){
    finalRad = arcTan;
  }
  else if(x > 0 && y < 0){
    finalRad = (2*M_PI) + arcTan; 
  } 
  else if(x < 0 && y > 0){
    finalRad = arcTan; //c
  }
  else if(x < 0 && y < 0) {
    finalRad =  (2*M_PI) + arcTan;
  }

  // finalRad = arcTan;

  return finalRad*(180/M_PI);
}