#include "thruster.h"

// Constructors

Motor::Motor() : position(0.0,0.0,0.0),kf(0.0),km(0.0),spin(1),maxRPM(0.0)
{}

Motor::Motor(Vector3 position,double kf,double km,int spin,double maxRPM) : position(position),
                                                                            kf(kf),
                                                                            km(km),
                                                                            spin(spin),
                                                                            maxRPM(maxRPM)
{}

// To run the motor

Vector3 Motor::runMotor(double RPM) const{
    return Vector3(0.0,0.0,kf*(RPM*RPM));   // RUns on the RPM provided
}

// Getter functions

Vector3 Motor::getPosition() const{
    return position;
}

double Motor::getKf() const{
    return kf;
}

double Motor::getKm() const{
    return km;
}

int Motor::getSpin() const{
    return spin;
}

double Motor::getMaxRPM() const{
    return maxRPM;
}

// Some functions to run and initialise the motors

// Creating all four motors of the drone and storing them in a MotorSet
MotorSet initialiseDrone(double armLength,double kf, double km, double maxRPM){

    Motor frontMotor(Vector3(0.0,armLength,0.0),kf,km,1,maxRPM);
    Motor rightMotor(Vector3(armLength,0.0,0.0),kf,km,-1,maxRPM);
    Motor backMotor(Vector3(0.0,-armLength,0.0),kf,km,1,maxRPM);
    Motor leftMotor(Vector3(-armLength,0.0,0.0),kf,km,-1,maxRPM);

    return {frontMotor,rightMotor,backMotor,leftMotor};
}

// Calculating the net thrust based upon the RPMs and the motor datas
Vector3 calcThrust(const std::array<double,4>& RPMs,const MotorSet& motors){

    Vector3 fT=motors.frontMotor.runMotor(RPMs[0]);
    Vector3 rT=motors.rightMotor.runMotor(RPMs[1]);
    Vector3 bT=motors.backMotor.runMotor(RPMs[2]);
    Vector3 lT=motors.leftMotor.runMotor(RPMs[3]);

    return fT+rT+bT+lT;
}

// Calculating the net torque based upon the RPMs and the motor datas
Vector3 calcTorque(const std::array<double,4>& RPMs,const MotorSet& motors){

    Vector3 fT=motors.frontMotor.runMotor(RPMs[0]);
    Vector3 rT=motors.rightMotor.runMotor(RPMs[1]);
    Vector3 bT=motors.backMotor.runMotor(RPMs[2]);
    Vector3 lT=motors.leftMotor.runMotor(RPMs[3]);

    Vector3 fTou=motors.frontMotor.getPosition()^fT; 
    Vector3 rTou=motors.rightMotor.getPosition()^rT; 
    Vector3 bTou=motors.backMotor.getPosition()^bT; 
    Vector3 lTou=motors.leftMotor.getPosition()^lT; 

    Vector3 resfTou(0.0,0.0,(motors.frontMotor.getSpin()*motors.frontMotor.getKm()*(RPMs[0]*RPMs[0])));
    Vector3 resrTou(0.0,0.0,(motors.rightMotor.getSpin()*motors.rightMotor.getKm()*(RPMs[1]*RPMs[1])));
    Vector3 resbTou(0.0,0.0,(motors.backMotor.getSpin()*motors.backMotor.getKm()*(RPMs[2]*RPMs[2])));
    Vector3 reslTou(0.0,0.0,(motors.leftMotor.getSpin()*motors.leftMotor.getKm()*(RPMs[3]*RPMs[3])));

    return fTou+rTou+bTou+lTou+resfTou+resrTou+resbTou+reslTou;
}