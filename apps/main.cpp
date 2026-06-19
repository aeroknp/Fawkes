#include <iostream>
#include <fstream>
#include <string>
#include "state.h"
#include "target.h"
#include "thruster.h"
#include "cascade.h"
#include "controller.h"
#include "rk4.h"
#include "dynamics.h"
#include "logger.h"
using namespace std;

int main(){

    // Initialising the state and time variables
    State state={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double t = 0.0;
    double tFinal = 1000.0;
    double dt = 0.01;

    // Initialising the variables for target object
    State targetState = {80.0,70.0,3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    targetParas target = {Vector3(targetState.x,targetState.y,targetState.z),Vector3(targetState.x,targetState.y,targetState.z),dt};

    // Initialising the drone 
    double armLength=0.2, kf=2.5e-7, km=5e-9, maxRPM=8000.0;
    MotorSet motors = initialiseDrone(armLength,kf,km,maxRPM);
    array<double,4> RPMs={0.0,0.0,0.0,0.0};

    // Initialsing the cascade controller
    Vector3 kpOuter(0.12,0.12,0.12), kpInner(2.0,2.0,2.0), kdInner(0.01,0.01,0.01), kiInner(0.02,0.02,0.02), maxI(0.1,0.1,0.1);
    Cascade cascadeController(kpOuter,kpInner,kdInner,kiInner,maxI);

    // Initialising the interception parameters
    double strikeRange = 0.1;
    double missDis = (Vector3(state.x,state.y,state.z)-target.targetState).magnitude();

    // Initialising the csv file to store data
    ofstream file;
    string filename;
    cout<<"Enter the filename to store the simulation data (without extension): ";
    cin>>filename;
    logger_init(file,filename);
    logState(file,t,state,target.targetState,missDis,RPMs);

    // Propagation loop
    while(t<tFinal){

        // Updating the target object
        target.prevTargetState=target.targetState;                               // Updating the previous target vector to current target vector
        runTarget(t,targetState,dt);                                             // Updating the current target state
        target.targetState=Vector3(targetState.x,targetState.y,targetState.z);   // Storing the current target state to current target vector

        // Getting the RPMs from the controller based on the relative target location
        RPMs = controllerOutput(t,state,target,motors,cascadeController);
        
        // Updating the state
        rk4Step(state,t,dt,derivatives,RPMs,motors);

        // Updating the miss distance
        missDis = (Vector3(state.x,state.y,state.z)-target.targetState).magnitude();

        // Updating time and logging the data
        t+=dt;
        logState(file,t,state,target.targetState,missDis,RPMs);

        // If strikes ground or target then stop the simulation
        if(state.z<0.0){
            cout<<endl<<"Crashed";
            break;
        }
        else if (missDis<strikeRange){
            cout<<endl<<"Target Intercepted";
            break;
        }
    }

    // Closing the file
    file.close();

    cout<<"\n\nSimulation Ended\n\n";

    return 0;
}