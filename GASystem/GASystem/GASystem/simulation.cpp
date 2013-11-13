#include "simulation.h"

Simulation::Simulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager){
    mNumCycles = _numCycles;
    mCyclesPerDecision = _cyclesPerDecision;
    mCycleCounter = 0;
    mCyclesPerSecond = _cyclesPerSecond;
    mInitialised = false;
    mSolution = _solution;
    mResourceManager = _resourceManager;

    mBroadphase = new btDbvtBroadphase();
    mCollisionConfig = new btDefaultCollisionConfiguration();
    mDispatcher = new btCollisionDispatcher(mCollisionConfig);
    mSolver = new btSequentialImpulseConstraintSolver();

    mBroadphaseEnv = new btDbvtBroadphase();
    mCollisionConfigEnv = new btDefaultCollisionConfiguration();
    mDispatcherEnv = new btCollisionDispatcher(mCollisionConfig);
    mSolverEnv = new btSequentialImpulseConstraintSolver();

    mWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
    mWorldEnv = new btDiscreteDynamicsWorld(mDispatcherEnv, mBroadphaseEnv, mSolverEnv, mCollisionConfigEnv);
}

Simulation::Simulation(const Simulation& other){}

Simulation::~Simulation(){
    for(map<string, Agent*>::const_iterator iter = mWorldEntities.begin(); iter != mWorldEntities.end(); iter++){
        if(iter->first == "environment")
            mWorldEnv->removeRigidBody(iter->second->getRigidBody());
        else mWorld->removeRigidBody(iter->second->getRigidBody());

        delete iter->second;
    }

    for(uint k = 0; k < mFitnessFunctions.size(); k++)
        delete mFitnessFunctions[k];
    mFitnessFunctions.clear();

    if(mWorld){
        delete mWorld;
        mWorld = 0;
    }

    if(mWorldEnv){
        delete mWorldEnv;
        mWorldEnv = 0;
    }
    
    if(mSolver){
        delete mSolver;
        mSolver = 0;
    }

    if(mSolverEnv){
        delete mSolverEnv;
        mSolverEnv = 0;
    }
    
    if(mDispatcher){
        delete mDispatcher;
        mDispatcher = 0;
    }

    if(mDispatcherEnv){
        delete mDispatcherEnv;
        mDispatcherEnv = 0;
    }
    
    if(mCollisionConfig){
        delete mCollisionConfig;
        mCollisionConfig = 0;
    }

    if(mCollisionConfigEnv){
        delete mCollisionConfigEnv;
        mCollisionConfigEnv = 0;
    }
    
    if(mBroadphase){
        delete mBroadphase;
        mBroadphase = 0;
    }

    if(mBroadphaseEnv){
        delete mBroadphaseEnv;
        mBroadphaseEnv = 0;
    }

}

uint Simulation::getCyclesPerSecond(){
    return mCyclesPerSecond;
}

void Simulation::setSolution(Solution* _solution){
    mSolution = _solution;
}

bool Simulation::isInitialised(){
    return mInitialised;
}

void Simulation::runFullSimulation(){
    for(uint k = 0; k < mNumCycles; k++)
        iterate();
}

const map<string, Agent*>& Simulation::getSimulationState(){
    return mWorldEntities;
}

vector<string> Simulation::getRemoveList(){
    return vector<string>();
}

vector<Line> Simulation::getLines(){
    return vector<Line>();
}

vector3 Simulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}


double Simulation::getRayCollisionDistance(string _agentName, const btVector3& _ray, RaycastLevel _rclevel){
    double dist = 100;

    btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_agentName, _ray, _rclevel);

    vector3 from = getPositionInfo(_agentName);
    if(ray.hasHit())
        dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));

    return dist;
}

btCollisionWorld::ClosestRayResultCallback Simulation::calculateRay(string _agentName, const btVector3& _ray, RaycastLevel _rclevel){
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    if(_rclevel == ENVIRONMENT)
        mWorldEnv->rayTest(agentPosition, correctedRay, ray);
    else mWorld->rayTest(agentPosition, correctedRay, ray);

    return ray;
}
