#include <algorithm>
#include <cmath>
#include "mixer.h"
using namespace std;

// Converts the required torque and thrust values to individual RPMs
array<double,4> getRPM(double thrust,const Vector3& torque,const MotorSet& motors){
    
    // Getting c and arm length
    double c= motors.frontMotor.getKm()/motors.frontMotor.getKf();
    double d=motors.frontMotor.getPosition()[1];

    // Individual forces on each motor
    array<double,4> forces = {
                    0.25*thrust + (torque[0])/(2*d) + (torque[2]/(4*c)),
                    0.25*thrust - (torque[1])/(2*d) - (torque[2]/(4*c)),
                    0.25*thrust - (torque[0])/(2*d) + (torque[2]/(4*c)),
                    0.25*thrust + (torque[1])/(2*d) - (torque[2]/(4*c))
    };

    // Clamping the forces above zero while maintaining the balance
    double minforce=*min_element(forces.begin(),forces.end());
    if(minforce<0.0){
        for(int i=0;i<4;i++)
            forces[i]-=minforce;
    }

    // Getting the RPMs
    array<double,4> RPMs;
    for(int i=0;i<4;i++)
        RPMs[i]=sqrt(forces[i]/motors.frontMotor.getKf());

    // Clamping the RPMs to the maximum RPM while maintaining balance
    double maxRPM=*max_element(RPMs.begin(),RPMs.end());
    if(maxRPM>motors.frontMotor.getMaxRPM()){
        double sf=maxRPM/motors.frontMotor.getMaxRPM();
        for(int i=0;i<4;i++)
            RPMs[i]/=sf;
    }

    return RPMs;
}