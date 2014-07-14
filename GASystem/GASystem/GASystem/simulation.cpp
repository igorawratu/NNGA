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

    mWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
}

Simulation::Simulation(const Simulation& other){}

Simulation::~Simulation(){
    for(map<string, Agent*>::const_iterator iter = mWorldEntities.begin(); iter != mWorldEntities.end(); iter++){
        mWorld->removeRigidBody(iter->second->getRigidBody());

        delete iter->second;
    }

    for(uint k = 0; k < mFitnessFunctions.size(); k++)
        delete mFitnessFunctions[k];
    mFitnessFunctions.clear();

    if(mWorld){
        delete mWorld;
        mWorld = 0;
    }

    if(mSolver){
        delete mSolver;
        mSolver = 0;
    }

    if(mDispatcher){
        delete mDispatcher;
        mDispatcher = 0;
    }

    if(mCollisionConfig){
        delete mCollisionConfig;
        mCollisionConfig = 0;
    }

    if(mBroadphase){
        delete mBroadphase;
        mBroadphase = 0;
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
    vector3 from = getPositionInfo(_agentName);

    if(_rclevel == AGENT){
        btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_agentName, _ray);

        if(ray.hasHit())
            dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));
    }
    else{
        btCollisionWorld::AllHitsRayResultCallback ray = calculateAllhitsRay(_agentName, _ray);
        vector<double> hitDistances;
        for(uint k = 0; k < ray.m_collisionObjects.size(); ++k){
            if(ray.m_collisionObjects[k] == mWorldEntities["environment"]->getRigidBody()){
                btVector3 hitpoint = ray.m_hitPointWorld[k];
                double newHitDistance = from.calcDistance(vector3(hitpoint.getX(), hitpoint.getY(), hitpoint.getZ()));
                hitDistances.push_back(newHitDistance);
            }
        }

        for(uint k = 0; k < hitDistances.size(); ++k){
            dist = dist > hitDistances[k] ? hitDistances[k] : dist;
        }
    }

    return dist;
}

double Simulation::getRayCollisionDistance(string _agentName, const btVector3& _ray, RaycastLevel _rclevel, vector3 _offset){
    double dist = 100;
    vector3 from = getPositionInfo(_agentName);

    if(_rclevel == AGENT){
        btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_agentName, _ray, _offset);

        if(ray.hasHit())
            dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));
    }
    else{
        btCollisionWorld::AllHitsRayResultCallback ray = calculateAllhitsRay(_agentName, _ray, _offset);
        vector<double> hitDistances;
        for(uint k = 0; k < ray.m_collisionObjects.size(); ++k){
            if(ray.m_collisionObjects[k] == mWorldEntities["environment"]->getRigidBody()){
                btVector3 hitpoint = ray.m_hitPointWorld[k];
                double newHitDistance = from.calcDistance(vector3(hitpoint.getX(), hitpoint.getY(), hitpoint.getZ()));
                hitDistances.push_back(newHitDistance);
            }
        }

        for(uint k = 0; k < hitDistances.size(); ++k){
            dist = dist > hitDistances[k] ? hitDistances[k] : dist;
        }
    }

    return dist;
}

btCollisionWorld::ClosestRayResultCallback Simulation::calculateRay(string _agentName, const btVector3& _ray){
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

btCollisionWorld::AllHitsRayResultCallback Simulation::calculateAllhitsRay(string _agentName, const btVector3& _ray){
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::AllHitsRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

btCollisionWorld::ClosestRayResultCallback Simulation::calculateRay(string _agentName, const btVector3& _ray, vector3 _offset){
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentCenter = trans.getOrigin();
    btVector3 correctedOffset = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * btVector3(_offset.x, _offset.y, _offset.z);
    btVector3 agentPosition = agentCenter + correctedOffset;

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

btCollisionWorld::AllHitsRayResultCallback Simulation::calculateAllhitsRay(string _agentName, const btVector3& _ray, vector3 _offset){
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentCenter = trans.getOrigin();
    btVector3 correctedOffset = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * btVector3(_offset.x, _offset.y, _offset.z);
    btVector3 agentPosition = agentCenter + correctedOffset;

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::AllHitsRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

double Simulation::getRayCollisionDistanceNonEnv(string _agentName, const btVector3& _ray, const btCollisionObject*& _collidedObject, vector3& _hitpos){
    double dist = 200;

    vector3 from = getPositionInfo(_agentName);
    btCollisionWorld::AllHitsRayResultCallback ray = calculateAllhitsRay(_agentName, _ray);

    vector<double> hitDistances;
    bool found = false;
    btVector3 hitpoint;
    for(uint k = 0; k < ray.m_collisionObjects.size(); ++k){
        if(ray.m_collisionObjects[k] != mWorldEntities["environment"]->getRigidBody()){
            found = true;
            vector3 currHitpoint(ray.m_hitPointWorld[k].getX(), ray.m_hitPointWorld[k].getY(), ray.m_hitPointWorld[k].getZ());
            double newHitDistance = from.calcDistance(currHitpoint);

            if(dist > newHitDistance){
                _collidedObject = ray.m_collisionObjects[k];
                _hitpos = currHitpoint;
                dist = newHitDistance;
            }
        }
    }

    if(!found)
        _hitpos = vector3(100, 0, 100);

    return dist;
}

double Simulation::getRayCollisionDistance(string _agentName, const btVector3& _ray, const btCollisionObject*& _collidedObject, vector3& _hitpos){
    double dist = 200;

    btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_agentName, _ray);

    vector3 from = getPositionInfo(_agentName);
    if(ray.hasHit()){
        dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));
        _collidedObject = ray.m_collisionObject;
        _hitpos = vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ());
    }
    else _collidedObject = 0;

    return dist;
}

vector<CompetitiveFitness> Simulation::competitiveFitness(){
    return vector<CompetitiveFitness>();
}