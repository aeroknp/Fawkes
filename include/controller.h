// This file contains the controller used in the simulation
#pragma once
#include <string>
#include <array>
#include "state.h"
#include "target.h"
#include "thruster.h"
#include "cascade.h"

std::array<double,4> controllerOutput(double,const State&,const targetParas&,const MotorSet&,Cascade&);
void updateMissionPhase(const State&,const Vector3&);