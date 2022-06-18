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