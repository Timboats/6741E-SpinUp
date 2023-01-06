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
template<typename T>
class PIDController{
    T error = 0;
    T integral = 0;
    T prevError = 0;
    T output = 0;
    bool nonDifferenceError = false;

    public:
    PIDController(bool customError){
        nonDifferenceError = customError;
    }
    T calculateOutput(T kP, T kI, T kD, T upperWindupLimit, T sp, T pv){
        if(!nonDifferenceError){
            error = sp - pv;
        }
        
        integral = integral + error;

        if (error > upperWindupLimit){
            integral = 0;
        }
        output = (kP * error) + (kI*integral) + (kD*(error - prevError));
        prevError = error;
        return output;
    }
    T getError(){
        return error;
    }
    void setError(T error){
        this->error = error;
    }

};
