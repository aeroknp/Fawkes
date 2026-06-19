#include "target.h"
#include "rk4.h"

// This function returns the target derivatives used to simulate the motion of the target
State targetDerivatives(double t,const State& targetState){
    double omega=0.2;

    double dxdt=targetState.vx;
    double dydt=targetState.vy;
    double dzdt=targetState.vz;

    double dvxdt=-(omega*omega)*targetState.x;
    double dvydt=-(omega*omega)*targetState.y;
    double dvzdt=1.0;

    return {dxdt,dydt,dzdt,dvxdt,dvydt,dvzdt,0.0,0.0,0.0,0.0,0.0,0.0};
}

// Updates the target state
void runTarget(double t, State& targetState, double dt){
    rk4StepForTarget(targetState,t,dt,targetDerivatives);
    return;
}