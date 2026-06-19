// This file contains the motor data and the functions required to run them
#pragma once
#include <array>
#include "Vector3.h"

class Motor{
    
    Vector3 position;
    double kf,km;
    int spin;
    double maxRPM;

    public:
        // Constructors
        Motor();
        Motor(Vector3 position,double kf,double km, int spin, double maxRPM);

        // To run the motor
        Vector3 runMotor(double RPM) const;

        // Getter functions
        Vector3 getPosition() const;
        double getKf() const;
        double getKm() const;
        int getSpin() const;
        double getMaxRPM() const;

};

// Stroing the motors
struct  MotorSet{
    Motor frontMotor,rightMotor,backMotor,leftMotor;
};

// Some functions to run and initialize the motors
MotorSet initialiseDrone(double armLength,double kf,double km, double maxRPM);
Vector3 calcThrust(const std::array<double,4>&,const MotorSet& motors);
Vector3 calcTorque(const std::array<double,4>&,const MotorSet& motors);
