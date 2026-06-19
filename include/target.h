// This file contains the dyanmics of the target used in this simulation
#pragma once
#include "state.h"

State targetDerivatives(double,const State&);
void runTarget(double,State&,double);

struct targetParas{
    Vector3 targetState;
    Vector3 prevTargetState;
    double dt;
};