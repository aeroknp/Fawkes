// This file contains the guidance laws used in this simulation
#pragma once
#include <functional>
#include <string>
#include "state.h"
#include "target.h"

State guidance(const std::string&,double,const State&,const targetParas&);
State liftOffGuidance(double,const State&,const targetParas&);
State purePursuitGuidance(double,const State&,const targetParas&);
State propNavGuidance(double,const State&,const targetParas&);