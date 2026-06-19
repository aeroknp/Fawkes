// This file contains the state struct used in the simulation
#pragma once
#include <cmath>
#include "constants.h"

struct State
{
    double x,y,z;
    double vx,vy,vz;
    double phi,theta,psi;    
    double p,q,r;
};

inline State operator+(const State& a, const State& b){
    return {a.x+b.x,a.y+b.y,a.z+b.z,a.vx+b.vx,a.vy+b.vy,a.vz+b.vz,
            a.phi+b.phi,a.theta+b.theta,a.psi+b.psi,a.p+b.p,a.q+b.q,a.r+b.r};
}

inline State operator-(const State& a, const State& b){
    return {a.x-b.x,a.y-b.y,a.z-b.z,a.vx-b.vx,a.vy-b.vy,a.vz-b.vz,
            a.phi-b.phi,a.theta-b.theta,a.psi-b.psi,a.p-b.p,a.q-b.q,a.r-b.r};
}

inline State operator*(double m, const State& a){
    return {a.x*m,a.y*m,a.z*m,a.vx*m,a.vy*m,a.vz*m,a.phi*m,a.theta*m,a.psi*m,a.p*m,a.q*m,a.r*m};
}

inline State operator*(const State& a,double m){
    return {a.x*m,a.y*m,a.z*m,a.vx*m,a.vy*m,a.vz*m,a.phi*m,a.theta*m,a.psi*m,a.p*m,a.q*m,a.r*m};
}

inline State operator/(const State& a,double m){
    
    if(std::abs(m)<denMin){
        if(m<0)
            m=-denMin;
        else
            m=denMin;
    }

    return {a.x/m,a.y/m,a.z/m,a.vx/m,a.vy/m,a.vz/m,a.phi/m,a.theta/m,a.psi/m,a.p/m,a.q/m,a.r/m};
}