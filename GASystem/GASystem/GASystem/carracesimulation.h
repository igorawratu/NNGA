#ifndef CARRACESIMULATION_H
#define CARRACESIMULATION_H

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "staticworldagent.h"
#include "finishlinefitness.h"
#include "collisionfitness.h"
#include "winnerfitness.h"

#include <iostream>
#include <vector>
#include <math.h>

class CarRaceSimulation : public Simulation
{
public:
    CarRaceSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~CarRaceSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    virtual double realFitness();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        CarRaceSimulation* sim = (CarRaceSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

    virtual vector<Line> getLines(){
        vector<Line> finishLine;
        finishLine.push_back(mFinishLine);

        return finishLine;
    }

    virtual vector<CompetitiveFitness> competitiveFitness();
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName, uint groupNum);
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.x)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }

private:
    Line mFinishLine;
    vector<string> mAgents; 
    long mCollisions;
    double mRangefinderVals, mRangefinderRadius;
    int mSeed, mWinner;
    double mWinToExpectedDistance;
};

#endif