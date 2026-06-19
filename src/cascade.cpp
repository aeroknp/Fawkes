#include <algorithm>
#include "cascade.h"

// Constructors

Cascade::Cascade() : kpOuter(0.0,0.0,0.0),
                     kpInner(0.0,0.0,0.0),
                     kdInner(0.0,0.0,0.0),
                     kiInner(0.0,0.0,0.0),
                     maxI(0.0,0.0,0.0),
                     errorIntegral(0.0,0.0,0.0),
                     prevError(0.0,0.0,0.0)
{}

Cascade::Cascade(const Vector3& kpOuter,const Vector3& kpInner,const Vector3& kdInner,
                 const Vector3& kiInner,const Vector3& maxI) : kpOuter(kpOuter),            // Sets the outer kp gains
                                                               kpInner(kpInner),            // Sets the inner kp gains 
                                                               kdInner(kdInner),            // Sets the inner kd gains
                                                               kiInner(kiInner),            // Sets the inner ki gains
                                                               maxI(maxI),                  // Sets the max integral value allowed
                                                               errorIntegral(0.0,0.0,0.0),  // Sets the integral to zero at start
                                                               prevError(0.0,0.0,0.0)       // Sets the previous error to zero at start
{}

// Functions to run the controller 

// Runs the outer to give roll rates 
Vector3 Cascade::runOuterLoop(const Vector3& errorVec){
    return kpOuter.hadamard(errorVec);
}

// Running the entire cascade controller
Vector3 Cascade::runCascade(const Vector3& errorVec,const State& state,double dt){

    // Comparing the commanded roll rates from outer loop to current roll rates to get the error vector
    Vector3 error = runOuterLoop(errorVec) - Vector3(state.p,state.q,state.r);

    Vector3 dedt = (error-prevError)/dt;        // Using the previous error to get the derivative of the error

    errorIntegral=errorIntegral+error*dt;       // Integral term for the error
    for(int i=0;i<3;i++){                       
        errorIntegral[i]=std::clamp(errorIntegral[i],-maxI[i],maxI[i]);     // Clamping the integral value
    }

    Vector3 torque = kpInner.hadamard(error) + kdInner.hadamard(dedt) + kiInner.hadamard(errorIntegral);

    prevError=error;    // Setting the current error to previous error 

    return torque;
}