// This file contains the gravity function used in this simulation
#pragma once
#include "state.h"

inline State gravity(){
    // Approximmating g to remain constant in the operating range
    return {0.0,0.0,0.0,0.0,0.0,-gSurface,0.0,0.0,0.0,0.0,0.0,0.0};
}