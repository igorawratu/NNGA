#include "staticworldagent.h"

StaticWorldAgent::StaticWorldAgent(double _restitution, double _friction){
    mRestitution = _restitution;
    mFriction = _friction;
}

StaticWorldAgent::~StaticWorldAgent(){
    delete dynamic_cast<btBvhTriangleMeshShape*>(mRigidBody->getCollisionShape())->getMeshInterface();
}

void StaticWorldAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    cout << "Error: Static objects do not have Collision avoidance" << endl;
}


void StaticWorldAgent::update(const vector<double>& _nnOutput){
    //do not need to update static world objects
}

void StaticWorldAgent::tick(){
    //no need to conform anything per tick
}

vector3 StaticWorldAgent::getVelocity(){
    return vector3();
}

btCollisionShape* StaticWorldAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, true, true, vector3(0, 0, 0), mScale);
}

void StaticWorldAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(mRestitution);
    mRigidBody->setFriction(mFriction);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
}

btVector3 StaticWorldAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    return btVector3(0, 0, 0);
}