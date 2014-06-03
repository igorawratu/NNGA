#ifndef EVACUATIONSIMULATION_H
#define EVACUATIONSIMULATION_H

#include "simulation.h"
#include "humanagent.h"
#include "staticworldagent.h"
#include "common.h"
#include "finishlinefitness.h"

#include <vector>
#include <iostream>

using namespace std;

class EvacuationSimulation : public Simulation
{
public:
    EvacuationSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~EvacuationSimulation();
    EvacuationSimulation(const EvacuationSimulation& other);
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

private:
    void applyUpdateRules(string _agentName);

private:
    Line mExit;
    vector<string> mAgents;
    long mCollisions;
    int mSeed;
    double mRangefinderVals, mRangefinderRadius;

    Line mExit;
    vector<string> mAgents; 
    long mCollisions;
    int mSeed;
    double mRangefinderRadius, mRangefinderVals;
    vector<Line> mLines;
    double mAngularVelAcc;
};

#endif