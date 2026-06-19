// This file generates the RPM of each motor from the thrust and torque values
#pragma once
#include <array>
#include "Vector3.h"
#include "thruster.h"

std::array<double,4> getRPM(double,const Vector3&,const MotorSet&);
