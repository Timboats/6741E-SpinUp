#pragma once
#include "mathlib.h"

template<typename T>
class TBHController {
    T tbh = 0;
    T error = 0;
    T prevError = 0;
    T output = 0;

    public:
    TBHController(T initTBH){
        tbh = initTBH;
    }
    T calculateOutput(T kI, T sp, T pv){
        error = sp - pv;
        output += kI*error;

        if(Simpler::sign(error) != Simpler::sign(prevError)){
            output = 0.5 * (output + tbh);
            tbh = output;
            prevError = error;
        }
        return output;
    }

};
