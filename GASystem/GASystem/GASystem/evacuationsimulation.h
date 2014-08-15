#ifndef EVACUATIONSIMULATION_H
#define EVACUATIONSIMULATION_H

#include "simulation.h"
#include "humanagent.h"
#include "staticworldagent.h"
#include "common.h"
#include "finishlinefitness.h"
#include "collisionfitness.h"

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
        EvacuationSimulation* sim = (EvacuationSimulation*)world->getWorldUserInfo();
        sim->tick();
    }
    virtual vector<Line> getLines();
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);
    virtual CMAESParameters getCMAESParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName, uint _group);
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.x)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }
    vector3 calculateAverageVelocity(string _agentName, double _radius);
    double calculateDensity(string _agentName, double _radius);

private:
    Line mExit;
    vector<string> mAgents; 
    long mCollisions;
    int mSeed;
    double mRangefinderRadius, mRangefinderVals;
    vector<Line> mLines;
    double mAngularVelAcc;
};

#endif