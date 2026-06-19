#include "environment.h"

// Getting the drag on the body
State drag(double t,const State& state,double rho,double Cd, double Area){
    
    double v = sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz);

    double fd = 0.5*(rho)*(v*v)*(Cd)*(Area);

    double dvxdt,dvydt,dvzdt;

    if(v!=0.0){
        dvxdt= -(fd/mass)*(state.vx/v);
        dvydt= -(fd/mass)*(state.vy/v);
        dvzdt= -(fd/mass)*(state.vz/v);
    }
    else{
        dvxdt=0.0;
        dvydt=0.0;
        dvzdt=0.0;
    }

    return {0.0,0.0,0.0,dvxdt,dvydt,dvzdt,0.0,0.0,0.0,0.0,0.0,0.0};
}