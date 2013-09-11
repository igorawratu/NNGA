#ifndef PROTOTYPESIMULATION_H
#define PROTOTYPESIMULATION_H

#include "simulation.h"
#include "common.h"
#include "cubeagent.h"
#include "staticworldagent.h"
#include "caragent.h"

#include <vector>
#include <map>

using namespace std;

class PrototypeSimulation : public Simulation
{
public:
    PrototypeSimulation(vector<vector3> _waypoints, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager);
    virtual ~PrototypeSimulation();
    virtual void iterate();
    virtual double fitness(vector<Fitness*> _fit);
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void conformVelocities();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        PrototypeSimulation* sim = (PrototypeSimulation*)world->getWorldUserInfo();
        sim->conformVelocities();
    }

private:
    void getRayCollisionDistances(map<uint, double>& _collisionDistances, const btVector3& _agentPosition);
    void applyUpdateRules(string _agentName);
    double calcDistance(vector3 _from, vector3 _to);
    vector3 getPositionInfo(string _entityName);

private:
    vector<vector3> mWaypoints;
    map<string, long> mWaypointTracker;
    long mCollisions;
};

#endif