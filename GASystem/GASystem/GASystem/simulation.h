#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"
#include "fitness.h"
#include "resourcemanager.h"
#include "solution.h"
#include "agent.h"
#include "standardgaparameters.h"
#include "espparameters.h"

#include <map>
#include <vector>
#include <string>
#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <boost/tuple/tuple.hpp>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

typedef boost::tuples::tuple<btRigidBody*, string, vector3> ObjectInfo;

using namespace std;

enum RaycastLevel{
    ENVIRONMENT = BIT(0),
    AGENT = BIT(1)
};

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
    virtual vector<CompetitiveFitness> competitiveFitness();
    virtual Simulation* getNewCopy()=0;

    const map<string, Agent*>& getSimulationState();

    virtual vector<string> getRemoveList();
    virtual vector<Line> getLines();

	virtual ESPParameters getESPParams(string _nnFormatFile)=0;
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile)=0;

protected:
    vector3 getPositionInfo(string _entityName);
    double getRayCollisionDistance(string _agentName, const btVector3& _ray, RaycastLevel _rclevel);
    double getRayCollisionDistance(string _agentName, const btVector3& _ray, RaycastLevel _rclevel, vector3 _offset);
    double getRayCollisionDistance(string _agentName, const btVector3& _ray, const btCollisionObject*& _collidedObject, vector3& _hitpos);
    double getRayCollisionDistanceNonEnv(string _agentName, const btVector3& _ray, const btCollisionObject*& _collidedObject, vector3& _hitpos);
    btCollisionWorld::ClosestRayResultCallback calculateRay(string _agentName, const btVector3& _ray);
    btCollisionWorld::ClosestRayResultCallback calculateRay(string _agentName, const btVector3& _ray, vector3 _offset);
    btCollisionWorld::AllHitsRayResultCallback calculateAllhitsRay(string _agentName, const btVector3& _ray);
    btCollisionWorld::AllHitsRayResultCallback calculateAllhitsRay(string _agentName, const btVector3& _ray, vector3 _offset);

    
protected:
    bool mInitialised;
    uint mNumCycles;
    uint mCyclesPerDecision;
    uint mCycleCounter;
    uint mCyclesPerSecond;
    map<string, Agent*> mWorldEntities;

    btBroadphaseInterface *mBroadphase;
    btDefaultCollisionConfiguration *mCollisionConfig;
    btCollisionDispatcher *mDispatcher;
    btSequentialImpulseConstraintSolver *mSolver;
    btDiscreteDynamicsWorld *mWorld;
    Solution* mSolution;
    ResourceManager* mResourceManager;

    vector<Fitness*> mFitnessFunctions;
};

#endif