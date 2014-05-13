#include "starfighteragent.h"

StarFighterAgent::StarFighterAgent(double _maxLinearVel, double _maxAngularVel){
    mMaxLinearVel = _maxLinearVel;
    mMaxAngularVel = _maxAngularVel;
    mCurrVel = 0;
}

void StarFighterAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    double left = getRayCollisionDistance(btVector3(100, 0, -10), _world, _envRigidBody);
    double right = getRayCollisionDistance(btVector3(100, 0, 10), _world, _envRigidBody);
    double bot = getRayCollisionDistance(btVector3(100, -10, 0), _world, _envRigidBody);
    double top = getRayCollisionDistance(btVector3(100, 10, 0), _world, _envRigidBody);

    double horizontalTorque, verticleTorque;

    //calculate rotation
    if((left < _frontRayDistance && right < _frontRayDistance) || (top < _frontRayDistance && bot < _frontRayDistance))
        mAvoidanceMode = true;
    
    if(mAvoidanceMode){
        btVector3 torque = btVector3(0, -0.75, 0);
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;

        mRigidBody->applyTorque(correctedTorque);
    }
    else{
        if(right > left)
            horizontalTorque = -0.75;
        else
            horizontalTorque = 0.75;

        if(bot > top)
            verticleTorque = -0.75;
        else
            verticleTorque = 0.75;

        btVector3 torque(0, horizontalTorque, verticleTorque);
        
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;
        mRigidBody->applyTorque(correctedTorque);
    }

    //calculate velocity
    /*double decisionsPerSecond = _cyclesPerSecond / _cyclesPerDecision;
    double deceleration = (-(mCurrVel * mCurrVel) / (2*_frontRayDistance - 3)) / decisionsPerSecond;
    deceleration = deceleration < 0 ? deceleration : 0;

    mCurrVel += deceleration;
    if(mCurrVel < 0)
        mCurrVel = 0;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);*/
}


void StarFighterAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 4);

    mAvoidanceMode = false;

    btVector3 torque((_nnOutput[0] - 0.5)/2, (_nnOutput[1] - 0.5)/2, (_nnOutput[2] - 0.5)/2);
    btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;
    mRigidBody->applyTorque(correctedTorque);

    double currAcc = (_nnOutput[3]) * 2.5;

    mCurrVel += currAcc * 10;
    if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;
    else if(mCurrVel < 0) mCurrVel = 0;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;

    mRigidBody->setLinearVelocity(correctedVel);
}

void StarFighterAgent::tick(){
    btVector3 currAngVel = mRigidBody->getAngularVelocity();

    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }

    btVector3 currLinVel = mRigidBody->getLinearVelocity();

    mCurrVel = vector3(0, 0, 0).calcDistance(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));
}

vector3 StarFighterAgent::getVelocity(){
    btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
    return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
}

btCollisionShape* StarFighterAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void StarFighterAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.7);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
}

btVector3 StarFighterAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}