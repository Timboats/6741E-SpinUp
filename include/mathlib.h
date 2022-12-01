#pragma once

class Formula{
    public:
    static float twoCoordDistance(float x1, float y1, float x2, float y2);
    static float findComponentVector(float magnitudeOne, float angleOne, float angleTwo);

};
class Simpler{
    public:
    static double degreeToStdPos(double angle);
    static float abs(float val);
    static int sign(float val);
    static double coterminalToStdPos(double degree);

};