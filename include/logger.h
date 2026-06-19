// This file writes the simulation data in the csv files
#pragma once
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <array>
#include "state.h"
#include "Vector3.h"

inline void logger_init(std::ofstream& file,std::string filename){

    std::filesystem::create_directories("data");
    filename="data/"+filename+".csv";
    file.open(filename);

    if (!file.is_open())
        throw std::runtime_error("Failed to open data logging file");

    file<<"t,x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,xtarget,ytarget,ztarget,missDis,fRPM,rRPM,bRPM,lRPM\n";
}

inline void logState(std::ofstream& file,double t, const State& s,const Vector3& ts,double missDis,const std::array<double,4>& RPMs){
    file<<t<<","<<s.x<<","<<s.y<<","<<s.z<<","<<s.vx<<","<<s.vy<<","<<s.vz<<
    ","<<s.phi<<","<<s.theta<<","<<s.psi<<","<<s.p<<","<<s.q<<","<<s.r<<","<<
    ts[0]<<","<<ts[1]<<","<<ts[2]<<","<<missDis<<","<<RPMs[0]<<","<<RPMs[1]<<","<<RPMs[2]<<","<<RPMs[3]<<"\n";
}