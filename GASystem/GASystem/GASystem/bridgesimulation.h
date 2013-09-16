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

enum AgentType{MOUSE, CAR};

//#define MT

class BridgeSimulation : public Simulation
{
public:
    BridgeSimulation(double _rangerfinderRadius, uint _numAgents, Line _finishLine, AgentType _agentType, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~BridgeSimulation();
    virtual void iterate();
    virtual double fitness(vector<Fitness*> _fit);
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
    double calcDistance(vector3 _from, vector3 _to);
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