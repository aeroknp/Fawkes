// This file contains the transformation matrices used accros the simulation
#pragma once
#include "Vector3.h"
#include "Matrix.h"

Matrix bodyToGlobal(double phi,double theta,double psi);
Matrix globalToBody(double phi,double theta,double psi);
Matrix rollRateBodyToEuler(double phi,double theta,double psi);