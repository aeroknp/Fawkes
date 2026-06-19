// This file contains rk4 step for integrating the simulation
#pragma once
#include <functional>
#include <array>
#include "state.h"
#include "thruster.h"

void rk4Step(State&,double,double,std::function<State(double,const State&,
            const std::array<double,4>&,const MotorSet&)>,const std::array<double,4>&,
            const MotorSet&);

void rk4StepForTarget(State&,double,double,std::function<State(double,const State&)>);
