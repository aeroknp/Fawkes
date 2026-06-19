#include <cmath>
#include "transformations.h"
#include "Vector3.h"
#include "Matrix.h"
using namespace std;

// Converts vector from body to global state
Matrix bodyToGlobal(double phi,double theta, double psi){

    double sphi=sin(phi);
    double cphi=cos(phi);

    double stheta=sin(theta);
    double ctheta=cos(theta);
    
    double spsi=sin(psi);
    double cpsi=cos(psi);

    return Matrix(ctheta*cpsi,sphi*stheta*cpsi-cphi*spsi,cphi*stheta*cpsi+sphi*spsi,
                  ctheta*spsi,sphi*stheta*spsi+cphi*cpsi,cphi*stheta*spsi-sphi*cpsi,
                  -stheta,sphi*ctheta,cphi*ctheta);
}

// Converts vector from global to body state
Matrix globalToBody(double phi, double theta, double psi){
    return bodyToGlobal(phi,theta,psi).transpose();
}

// Converts the roll rates from body state to euler state
Matrix rollRateBodyToEuler(double phi,double theta, double psi){

    double sphi=sin(phi);
    double cphi=cos(phi);

    double ctheta=cos(theta);
    double ttheta=tan(theta);

    return Matrix(1.0,sphi*ttheta,cphi*ttheta,
                  0.0,cphi,-sphi,
                  0.0,sphi/ctheta,cphi/ctheta);
}