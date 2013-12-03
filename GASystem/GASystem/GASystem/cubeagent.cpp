#include "cubeagent.h"

CubeAgent::CubeAgent(vector3 _maxVel, vector3 _minVel){
    mMaxVel = _maxVel;
    mMinVel = _minVel;
}

CubeAgent::~CubeAgent(){}

void CubeAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    cout << "Error: Collision avoidance not enabled on cube agents" << endl;
}


void CubeAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 2);

    double xAccel = (_nnOutput[0] - 0.5) * 10;
    double zAccel = (_nnOutput[1] - 0.5) * 10;

    btVector3 vel = mRigidBody->getLinearVelocity();
    mRigidBody->setLinearVelocity(btVector3(vel.getX() + xAccel, 0, vel.getZ() + zAccel));
}

vector3 CubeAgent::getVelocity(){
    btVector3 vel = mRigidBody->getLinearVelocity();
    return vector3(vel.getX(), vel.getY(), vel.getZ());
}

void CubeAgent::setPosition(vector3 _pos){
    delete mRigidBody->getMotionState();
    btMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(_pos.x, _pos.y, _pos.z)));
    mRigidBody->setMotionState(ms);
}

void CubeAgent::tick(){
    vector3 newVelocity;

    if(mRigidBody->getLinearVelocity().getX() > mMaxVel.x)
        newVelocity.x = mMaxVel.x;
    else if(mRigidBody->getLinearVelocity().getX() < mMinVel.x)
        newVelocity.x = mMinVel.x;
    else newVelocity.x = mRigidBody->getLinearVelocity().getX();

    if(mRigidBody->getLinearVelocity().getY() > mMaxVel.y)
        newVelocity.y = mMaxVel.y;
    else if(mRigidBody->getLinearVelocity().getY() < mMinVel.y)
        newVelocity.y = mMinVel.y;
    else newVelocity.y = mRigidBody->getLinearVelocity().getY();

    if(mRigidBody->getLinearVelocity().getZ() > mMaxVel.z)
        newVelocity.z = mMaxVel.z;
    else if(mRigidBody->getLinearVelocity().getZ() < mMinVel.z)
        newVelocity.z = mMinVel.z;
    else newVelocity.z = mRigidBody->getLinearVelocity().getZ();

    mRigidBody->setLinearVelocity(btVector3(newVelocity.x, newVelocity.y, newVelocity.z));
}

btCollisionShape* CubeAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void CubeAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.5);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
}

btVector3 CubeAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}