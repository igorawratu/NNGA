#include "prototypesimulation.h"

PrototypeSimulation::PrototypeSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution){
    mWorld->setGravity(btVector3(0, -1.f, 0));
    mWorld->setInternalTickCallback(PrototypeSimulation::tickCallBack, this, true);
}

PrototypeSimulation::~PrototypeSimulation(){}

void PrototypeSimulation::iterate(){
    /*if(mCycleCounter % mCyclesPerDecision == 0){
        applyUpdateRules("agentOne");
        applyUpdateRules("agentTwo");
    }*/

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double PrototypeSimulation::fitness(vector<Fitness*> _fit){
    
    return 0;
}

Simulation* PrototypeSimulation::getNewCopy(){
    return new PrototypeSimulation(mNumCycles, mCyclesPerDecision, mCyclesPerDecision, mSolution);
}

void PrototypeSimulation::conformVelocities(){
    vector3 maxVel(2, 2, 2), minVel(-2, -2, -2), agentOneVel,agentTwoVel;

    btRigidBody* agentOne = mWorldEntities["agentOne"].get<0>();
    btRigidBody* agentTwo = mWorldEntities["agentTwo"].get<0>();

    btVector3 velocity = agentOne->getLinearVelocity();
    bool update = false;

    //agent one conform
    if(agentOne->getLinearVelocity().getX() > maxVel.x)
        agentOneVel.x = maxVel.x;
    else if(agentOne->getLinearVelocity().getX() < minVel.x)
        agentOneVel.x = minVel.x;
    else agentOneVel.x = agentOne->getLinearVelocity().getX();

    if(agentOne->getLinearVelocity().getY() > maxVel.y)
        agentOneVel.y = maxVel.y;
    else if(agentOne->getLinearVelocity().getY() < minVel.y)
        agentOneVel.y = minVel.y;
    else agentOneVel.y = agentOne->getLinearVelocity().getY();

    if(agentOne->getLinearVelocity().getZ() > maxVel.z)
        agentOneVel.z = maxVel.z;
    else if(agentOne->getLinearVelocity().getZ() < minVel.z)
        agentOneVel.z = minVel.z;
    else agentOneVel.z = agentOne->getLinearVelocity().getZ();

    //agent two conform
    if(agentTwo->getLinearVelocity().getX() > maxVel.x)
        agentTwoVel.x = maxVel.x;
    else if(agentTwo->getLinearVelocity().getX() < minVel.x)
        agentTwoVel.x = minVel.x;
    else agentTwoVel.x = agentOne->getLinearVelocity().getX();

    if(agentTwo->getLinearVelocity().getY() > maxVel.y)
        agentTwoVel.y = maxVel.y;
    else if(agentTwo->getLinearVelocity().getY() < minVel.y)
        agentTwoVel.y = minVel.y;
    else agentTwoVel.y = agentTwo->getLinearVelocity().getY();

    if(agentTwo->getLinearVelocity().getZ() > maxVel.z)
        agentTwoVel.z = maxVel.z;
    else if(agentTwo->getLinearVelocity().getZ() < minVel.z)
        agentTwoVel.z = minVel.z;
    else agentTwoVel.z = agentTwo->getLinearVelocity().getZ();

    agentOne->setLinearVelocity(btVector3(agentOneVel.x, agentOneVel.y, agentOneVel.z));
    agentTwo->setLinearVelocity(btVector3(agentTwoVel.x, agentTwoVel.y, agentTwoVel.z));
}

bool PrototypeSimulation::initialise(ResourceManager* _rm){
    if(mInitialised)
        return true;

    //agents
    if(!createObject("cube.mesh", "agentOne", vector3(1, 1, 1), vector3(-50, 50, -10), 1, _rm))
        return false;

    cout << "created agent one" << endl;

    if(!createObject("cube.mesh", "agentTwo", vector3(1, 1, 1), vector3(10, 50, -10), 1, _rm))
        return false;

    cout << "created agent two" << endl;
    
    //maze
    if(!createObject("maze.mesh", "maze", vector3(10, 10, 10), vector3(0, 0, 0), 0, _rm))
        return false;

    cout << "created agent maze" << endl;

    mInitialised = true;
    
    return true;
}

bool PrototypeSimulation::createObject(string _meshname, string _entityName, vector3 _scale, vector3 _position, float _mass, ResourceManager* _rm){
    btConvexShape* shape = _rm->getBulletCollisionShape(_meshname, vector3(0, 0, 0), _scale);
    
    if(!shape)
        return false;

    btMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(_position.x, _position.y, _position.z)));
    
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(_mass, inertia);
    
    btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

    btRigidBody* rbody = new btRigidBody(constructionInfo);
    rbody->setSleepingThresholds(0.f, rbody->getAngularSleepingThreshold());
    mWorld->addRigidBody(rbody);
    mWorldEntities[_entityName] = ObjectInfo(rbody, _meshname, _scale);

    return true;
}

void PrototypeSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName].get<0>()->getMotionState()->getWorldTransform(trans);

    map<uint, double> input;
    getRayCollisionDistances(input, trans.getOrigin());

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName].get<0>()->applyCentralForce(btVector3(output[0], 0, output[1]));
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