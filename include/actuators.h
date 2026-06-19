// This file contains the actuators to command the motors
#pragma once
#include <array>
#include "state.h"
#include "thruster.h"

State actuatorDerivatives(const State&,const std::array<double,4>&,const MotorSet&);