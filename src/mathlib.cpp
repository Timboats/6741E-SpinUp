#include "mathlib.h"
#include <cmath>


float Formula::twoCoordDistance(float x1, float y1, float x2, float y2){
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
int Simpler::degreeToStdPos(int angle){
  return(((360 - angle) + 90) % 360);    
}