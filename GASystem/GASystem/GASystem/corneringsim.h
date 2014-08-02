#ifndef CORNERINGSIM_H
#define CORNERINGSIM_H

#include "simulation.h"
#include "caragent.h"
#include "staticworldagent.h"
#include "common.h"
#include "collisionfitness.h"
#include "waypointfitness.h"

#include <vector>
#include <iostream>
#include <omp.h>

using namespace std;

class CorneringSim : public Simulation
{
public:
    CorneringSim(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~CorneringSim();
    CorneringSim(const CorneringSim& other);
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        CorneringSim* sim = (CorneringSim*)world->getWorldUserInfo();
        sim->tick();
    }
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName);

private:
    vector<vector3> mWaypoints;
    vector<string> mAgents; 
    map<string, long> mWaypointTracker;
    long mCollisions;
    int mSeed;
    double mRangefinderVals, mRangefinderRadius;
    double mAngularVelAcc;
};

#endif