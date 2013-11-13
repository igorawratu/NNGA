#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"
#include "fitness.h"
#include "resourcemanager.h"
#include "solution.h"
#include "agent.h"

#include <map>
#include <vector>
#include <string>
#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <boost/tuple/tuple.hpp>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

typedef boost::tuples::tuple<btRigidBody*, string, vector3> ObjectInfo;

using namespace std;

enum RaycastLevel{AGENT, ENVIRONMENT};

class Simulation
{
public:
    Simulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager);
    Simulation(const Simulation& other);
    
    virtual ~Simulation();

    uint getCyclesPerSecond();

    virtual double realFitness()=0;

    void setSolution(Solution* _solution);

    virtual void iterate()=0;
    virtual bool initialise()=0;

    bool isInitialised();
    void runFullSimulation();

    virtual double fitness()=0;
    virtual Simulation* getNewCopy()=0;

    const map<string, Agent*>& getSimulationState();

    virtual vector<string> getRemoveList();
    virtual vector<Line> getLines();
protected:
    vector3 getPositionInfo(string _entityName);
    double getRayCollisionDistance(string _agentName, const btVector3& _ray, RaycastLevel _rclevel);
    btCollisionWorld::ClosestRayResultCallback calculateRay(string _agentName, const btVector3& _ray, RaycastLevel _rclevel);

    
protected:
    bool mInitialised;
    uint mNumCycles;
    uint mCyclesPerDecision;
    uint mCycleCounter;
    uint mCyclesPerSecond;
    map<string, Agent*> mWorldEntities;

    btBroadphaseInterface *mBroadphase, *mBroadphaseEnv;
    btDefaultCollisionConfiguration *mCollisionConfig, *mCollisionConfigEnv;
    btCollisionDispatcher *mDispatcher, *mDispatcherEnv;
    btSequentialImpulseConstraintSolver *mSolver, *mSolverEnv;
    btDiscreteDynamicsWorld *mWorld, *mWorldEnv;
    Solution* mSolution;
    ResourceManager* mResourceManager;

    vector<Fitness*> mFitnessFunctions;
};

#endif