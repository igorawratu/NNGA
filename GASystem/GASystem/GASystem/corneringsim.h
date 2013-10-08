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
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        CorneringSim* sim = (CorneringSim*)world->getWorldUserInfo();
        sim->tick();
    }

private:
    double getRayCollisionDistance(string _agentName, const btVector3& _ray);
    void applyUpdateRules(string _agentName);
    vector3 getPositionInfo(string _entityName);

private:
    vector<vector3> mWaypoints;
    vector<string> mAgents; 
    map<string, long> mWaypointTracker;
    long mCollisions;
    int mSeed;
    double mRangefinderVals, mRangefinderRadius;
};

#endif