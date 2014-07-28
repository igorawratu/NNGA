#ifndef MOUSESCATTERSIMULATION_H
#define MOUSESCATTERSIMULATION_H

#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>
#include <math.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "mouseagent.h"
#include "staticworldagent.h"
#include "mouseagent.h"
#include "collisionfitness.h"
#include "expectedvaluefitness.h"

class MouseScatterSimulation : public Simulation
{
public:
    MouseScatterSimulation(double _rangerfinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    MouseScatterSimulation(const MouseScatterSimulation& other);
    virtual ~MouseScatterSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    virtual vector<Line> getLines();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        MouseScatterSimulation* sim = (MouseScatterSimulation*)world->getWorldUserInfo();
        sim->tick();
    }
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName);

private:
    vector3 mCenterPoint;
    vector<string> mAgents; 
    vector<Line> mLines;
    long mCollisions;
    int mSeed;
    double mRangefinderRadius;
    double mRangefinderVals;
};

#endif