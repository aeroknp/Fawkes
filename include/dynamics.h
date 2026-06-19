// This file contains the dynamics file used in the simulation
#pragma once
#include <array>
#include "state.h"
#include "thruster.h"

State derivatives(double,const State&,const std::array<double,4>&,const MotorSet&);