#include "rk4.h"

// Writing the rk4 step function
void rk4Step(State& state,double t, double dt,std::function<State(double,const State&,
            const std::array<double,4>&,const MotorSet&)>derivatives,
            const std::array<double,4>& RPMs,const MotorSet& motors){
                
    State k1=derivatives(t,state,RPMs,motors);
    State k2=derivatives(t+(dt/2),state+k1*(dt/2),RPMs,motors);
    State k3=derivatives(t+(dt/2),state+k2*(dt/2),RPMs,motors);
    State k4=derivatives(t+dt,state+k3*(dt),RPMs,motors);

    state=state+(k1+2.0*k2+2.0*k3+k4)*(dt/6.0);
    return;
}

// Writing the rk4 step for target
void rk4StepForTarget(State& targetState,double t, double dt,std::function<State(double,const State&)> targetDerivatives){
    
    State k1=targetDerivatives(t,targetState);
    State k2=targetDerivatives(t+(dt/2),targetState+k1*(dt/2));
    State k3=targetDerivatives(t+(dt/2),targetState+k2*(dt/2));
    State k4=targetDerivatives(t+dt,targetState+k3*(dt));

    targetState=targetState+(k1+2.0*k2+2.0*k3+k4)*(dt/6.0);
    return;
}
