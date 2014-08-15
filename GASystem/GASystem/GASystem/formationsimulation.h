#ifndef FORMATIONSIMULATION_H
#define FORMATIONSIMULATION_H

#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "staticworldagent.h"
#include "collisionfitness.h"
#include "goalpointfitness.h"

class FormationSimulation : public Simulation
{
public:
    FormationSimulation(double _rangerfinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    FormationSimulation(const FormationSimulation& other);
    virtual ~FormationSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        FormationSimulation* sim = (FormationSimulation*)world->getWorldUserInfo();
        sim->tick();
    }
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);
    virtual CMAESParameters getCMAESParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName);

private:
    vector3 mCircleCenter;
    vector<string> mAgents; 
    long mCollisions;
    int mSeed;
    double mRangefinderRadius;
    double mRangefinderVals;
    double mCircleRadius;
};

#endif