#pragma once

struct coord{
    double x;
    double y;
};

class Formula{
    public:
    static double twoCoordDistance(double x1, double y1, double x2, double y2);
    static float findComponentVector(float magnitudeOne, float angleOne, float angleTwo);
    static coord findCircleIntersect(double x1, double y1, double x2, double y2, double radius);

};
class Simpler{
    public:
    static int degreeToStdPos(int angle);
    static double abs(double val);
    static int sign(float val);
    static double coterminalToStdPos(double degree);
    static double degAvgTwoAngles(double angle1, double angle2);

};

