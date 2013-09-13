#ifndef CORNERINGSIM_H
#define CORNERINGSIM_H

#include "simulation.h"
#include "caragent.h"
#include "staticworldagent.h"
#include "common.h"

#include <vector>
#include <iostream>

using namespace std;

class CorneringSim : public Simulation
{
public:
    CorneringSim(vector<vector3> _waypoints, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~CorneringSim();
    virtual void iterate();
    virtual double fitness(vector<Fitness*> _fit);
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
    double calcDistance(vector3 _from, vector3 _to);
    vector3 getPositionInfo(string _entityName);

private:
    vector<vector3> mWaypoints;
    vector<string> mAgents; 
    map<string, long> mWaypointTracker;
    long mCollisions;
    int mSeed;
};

#endif