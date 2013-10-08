#ifndef BRIDGESIMULATION_H
#define BRIDGESIMULATION_H

#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "mouseagent.h"
#include "staticworldagent.h"
#include "cubeagent.h"
#include "collisionfitness.h"
#include "finishlinefitness.h"

enum AgentType{MOUSE, CAR};

class BridgeSimulation : public Simulation
{
public:
    BridgeSimulation(double _rangerfinderRadius, uint _numAgents, AgentType _agentType, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    BridgeSimulation(const BridgeSimulation& other);
    virtual ~BridgeSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        BridgeSimulation* sim = (BridgeSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

private:
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.z)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }
    double getRayCollisionDistance(string _agentName, const btVector3& _ray);
    void applyUpdateRules(string _agentName);
    vector3 getPositionInfo(string _entityName);

private:
    Line mFinishLine;
    vector<string> mAgents; 
    long mCollisions;
    AgentType mAgentType;
    int mSeed;
    double mRangefinderRadius;
    double mRangefinderVals;
};

#endif