#include "controller.h"
#include "guidance.h"
#include "gravity.h"
#include "environment.h"
#include "transformations.h"
#include "mixer.h"
using namespace std;

string phase = "IDLE";

// This is the controller responsible to calculate the required RPMs at any point of time
array<double,4> controllerOutput(double t, const State& state, const targetParas& target,const MotorSet& motors,Cascade& cascadeController){

    // Updating the mission phase based on the current state and target state
    updateMissionPhase(state,target.targetState);

    // Getting the net acceleration required in the global frame
    State reqStateDeri = guidance(phase,t,state,target)-gravity()-drag(t,state,rhoSurface,CdTrue,AreaTrue);
    Vector3 anet(reqStateDeri.vx,reqStateDeri.vy,reqStateDeri.vz);

    // Converting to bdoy frame
    Vector3 Anet = globalToBody(state.phi,state.theta,state.psi)*anet;

    // Calculate magnitude of the required acceleration
    double AnetMag = Anet.magnitude();

    // Getting the error vector
    Vector3 errorVec(-Anet[1],Anet[0],0.0);

    // Getting the required torque based on the error vector
    Vector3 Torque = cascadeController.runCascade(errorVec,state,target.dt);

    // Returning the required RPMs based on the required torque and thrust
    return getRPM(AnetMag*mass,Torque,motors);
}

// This function updates the mission phase based on the current state and target state
void updateMissionPhase(const State& state, const targetParas& target){
    
    // Getting the variables
    Vector3 rT=target.targetState;
    Vector3 rM(state.x,state.y,state.z);
    Vector3 rTM = rT-rM;
    Vector3 vM(state.vx,state.vy,state.vz);

    double cosTheta = (vM*rTM)/max(vM.magnitude()*rTM.magnitude(),denMin);

    // If the drone is close to the ground and in IDLE or rise phase, swtich to rise phase
    if(state.z<0.5 && (phase=="IDLE" || phase=="rise"))
        phase="rise";
    
    // If the drone is in rise or pure pursuit phase
    else if (phase=="rise" || phase=="purePursuit"){
        // If the drone is well aligned witht the target then check its speed else stay in pure pursuit phase
        if(cosTheta>0.5){
            //  If the drone is moving fast enough, switch to propNavPhase, else swtich to pure pursuit phase
            if(vM.magnitude()>0.5)
                phase="propNav";
            else
                phase="purePursuit";
        }
        else
            phase="purePursuit";
    }
    
    // If the drone is in propNav phase then stay in it
    else
        phase="propNav";
}