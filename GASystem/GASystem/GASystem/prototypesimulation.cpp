#include "prototypesimulation.h"

PrototypeSimulation::PrototypeSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution){
}

PrototypeSimulation::~PrototypeSimulation(){}

void PrototypeSimulation::iterate(){
    applyUpdateRules("agentOne");
    applyUpdateRules("agentTwo");

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double PrototypeSimulation::fitness(vector<Fitness*> _fit){
    
    return 0;
}

Simulation* PrototypeSimulation::getNewCopy(){
    return new PrototypeSimulation(mNumCycles, mCyclesPerDecision, mCyclesPerDecision, mSolution);
}

bool PrototypeSimulation::initialise(ResourceManager* _rm){
    if(mInitialised)
        return true;

    //agents
    if(!createObject("cube.mesh", "agentOne", vector3(1, 1, 1), vector3(0, 10, -10), 1, _rm))
        return false;

    if(!createObject("cube.mesh", "agentTwo", vector3(1, 1, 1), vector3(0, -10, -10), 1, _rm))
        return false;
    
    //create maze here

    mInitialised = true;
    
    return true;
}

bool PrototypeSimulation::createObject(string _meshname, string _entityName, vector3 _scale, vector3 _position, float _mass, ResourceManager* _rm){
    btConvexShape* shape = _rm->getBulletCollisionShape(_meshname, vector3(0, 0, 0), Ogre::Quaternion::IDENTITY, _scale);
    if(!shape)
        return false;

    btMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(_position.x, _position.y, _position.z)));
    
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(_mass, inertia);
    
    btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

    btRigidBody* rbody = new btRigidBody(constructionInfo);
    mWorld->addRigidBody(rbody);
    mWorldEntities[_entityName] = make_pair(rbody, _meshname);
}

void PrototypeSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName].first->getMotionState()->getWorldTransform(trans);

    map<uint, double> input;
    getRayCollisionDistances(input, trans.getOrigin());

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName].first->applyCentralForce(btVector3(output[0], 0, output[1]));
}

void PrototypeSimulation::getRayCollisionDistances(map<uint, double>& _collisionDistances, const btVector3& _agentPosition){
    btVector3 toTop(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ() + 2000);
    btVector3 toLeft(_agentPosition.getX() - 2000, _agentPosition.getY(), _agentPosition.getZ());
    btVector3 toRight(_agentPosition.getX() + 2000, _agentPosition.getY(), _agentPosition.getZ());
    btVector3 toBot(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ() - 2000);

    btCollisionWorld::ClosestRayResultCallback topRay(_agentPosition, toTop);
    btCollisionWorld::ClosestRayResultCallback leftRay(_agentPosition, toLeft);
    btCollisionWorld::ClosestRayResultCallback rightRay(_agentPosition, toRight);
    btCollisionWorld::ClosestRayResultCallback botRay(_agentPosition, toBot);

    mWorld->rayTest(_agentPosition, toTop, topRay);
    mWorld->rayTest(_agentPosition, toBot, botRay);
    mWorld->rayTest(_agentPosition, toLeft, leftRay);
    mWorld->rayTest(_agentPosition, toRight, rightRay);

    vector3 from(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ());
    if(topRay.hasHit()){
        vector3 to(topRay.m_hitPointWorld.getX(), topRay.m_hitPointWorld.getY(), topRay.m_hitPointWorld.getZ());
        _collisionDistances[1] = calcEucDistance(from, to);
    }
    else _collisionDistances[1] = 2000;

    if(botRay.hasHit()){
        vector3 to(botRay.m_hitPointWorld.getX(), botRay.m_hitPointWorld.getY(), botRay.m_hitPointWorld.getZ());
        _collisionDistances[1] = calcEucDistance(from, to);
    }
    else _collisionDistances[2] = 2000;

    if(leftRay.hasHit()){
        vector3 to(leftRay.m_hitPointWorld.getX(), leftRay.m_hitPointWorld.getY(), leftRay.m_hitPointWorld.getZ());
        _collisionDistances[3] = calcEucDistance(from, to);
    }
    else _collisionDistances[3] = 2000;

    if(rightRay.hasHit()){
        vector3 to(rightRay.m_hitPointWorld.getX(), rightRay.m_hitPointWorld.getY(), rightRay.m_hitPointWorld.getZ());
        _collisionDistances[4] = calcEucDistance(from, to);
    }
    else _collisionDistances[4] = 2000;
}