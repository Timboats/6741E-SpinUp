#include "mathlib.h"
#include <cmath>


double Formula::twoCoordDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
int Simpler::degreeToStdPos(int angle){
  return(((360 - angle) + 90) % 360);    
}
double Simpler::abs(double val){
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
coord Formula::findCircleIntersect(double x1, double y1, double x2, double y2, double radius){
  //TODO make this return a point struct and return null if no intersections are found

  coord noResult;
  noResult.x = -2000;
  noResult.y = -2000;

  //change in x and y
  const double dx = x2-x1;
  const double dy = y2-y1;

  const double dr = twoCoordDistance(x1, y1, x2, y2);

  const double dMatrix = (x1*y2)-(x2*y1);

  const double discriminant = (pow(radius, 2)*pow(dr, 2)) - pow(dMatrix, 2);

  if(discriminant < 0){
    return noResult;
  }

  const double xPreCanidate = Simpler::sign(dy)*dx*sqrt(discriminant);
  const double yPreCanidate = Simpler::abs(dy)*sqrt(discriminant);

  const double xCanidate1 = ((dMatrix*dy)+xPreCanidate)/pow(dr, 2);
  const double xCanidate2 = ((dMatrix*dy)-xPreCanidate)/pow(dr, 2);

  const double yCanidate1 = (-(dMatrix*dx)+yPreCanidate)/pow(dr, 2);
  const double yCanidate2 = (-(dMatrix*dx)-yPreCanidate)/pow(dr, 2);

  coord can1Point;
  can1Point.x = xCanidate1;
  can1Point.y = yCanidate1;

  coord can2Point;
  can2Point.x = xCanidate2;
  can2Point.y = yCanidate2;

  const double minX = (x1<x2)?x1:x2;
  const double minY = (y1<y2)?y1:y2;
  const double maxX = (x1>x2)?x1:x2;
  const double maxY = (y1>y2)?y1:y2;

  if((xCanidate1 == xCanidate2) && (yCanidate1 == yCanidate2)){
    if((minX<=xCanidate1<=maxX) && (minY<=yCanidate1<=maxY)){
      return can1Point;
    } 
    else{
      return noResult;
    }
  }

  const double canidate1Distance = Formula::twoCoordDistance(x2, y2, xCanidate1, yCanidate1);
  const double canidate2Distance = Formula::twoCoordDistance(x2, y2, xCanidate2, yCanidate2);

  

  if(canidate1Distance < canidate2Distance){
    if((minX<=xCanidate1<=maxX) && (minY<=yCanidate1<=maxY)){
      return can1Point;

    }
  }
  else {
    if((minX<=xCanidate2<=maxX) && (minY<=yCanidate2<=maxY)){
      return can2Point;

    }
  }

  return noResult;
}