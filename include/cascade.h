// This file contains the cascade controller used in the simulation
#pragma once
#include "Vector3.h"
#include "state.h"

class Cascade{
    // Parameters
    Vector3 kpOuter;
    Vector3 kpInner;
    Vector3 kdInner;
    Vector3 kiInner;
    Vector3 maxI;
    Vector3 errorIntegral;
    Vector3 prevError;

    public:
        // Constructors
        Cascade();
        Cascade(const Vector3&,const Vector3&,const Vector3&,const Vector3&,const Vector3&);

        // Functions to run the controller
        Vector3 runOuterLoop(const Vector3&);
        Vector3 runCascade(const Vector3&,const State&,double);
};