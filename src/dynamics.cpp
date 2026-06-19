#include <random>
#include "dynamics.h"
#include "kinematics.h"
#include "environment.h"
#include "gravity.h"
#include "actuators.h"

// Creating the module to generate random Cd and Area values

// Create generator once
std::mt19937 gen(std::random_device{}());

// Create distributions
std::uniform_real_distribution<double> cdDist(CdLower, CdUpper);
std::uniform_real_distribution<double> areaDist(AreaLower, AreaUpper);

// Generate random values
double Cd = cdDist(gen);
double Area = areaDist(gen);

// Getting the total derivatives function
State derivatives(double t, const State& state,const std::array<double,4>& RPMs,const MotorSet& motors){
    
    State total = kinematics(t,state)+drag(t,state,rhoSurface,Cd,Area)+gravity()+actuatorDerivatives(state,RPMs,motors);

    return total;
}