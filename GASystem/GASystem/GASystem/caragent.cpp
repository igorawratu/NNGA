#include "caragent.h"

CarAgent::CarAgent(double _maxLinearVel, double _maxAngularVel){
    mMaxLinearVel = _maxLinearVel;
    mMaxAngularVel = _maxAngularVel;
    mCurrVel = 0;
}

void CarAgent::avoidCollisions(double _distanceLeft, double _distanceRight, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    double left = _distanceLeft;
    double right = _distanceRight;
    double _frontRayDistance = left < right ? left : right;

    if(_frontRayDistance < 1.5)
        _frontRayDistance = 1.5001;

    //calculate rotation
    if(left < _frontRayDistance && right < _frontRayDistance)
        mAvoidanceMode = true;
    
    if(mAvoidanceMode){
        btVector3 torque = btVector3(0, -0.5, 0);
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;

        mRigidBody->applyTorque(correctedTorque);
    }
    else{
        btVector3 torque;

        if(right > left)
            torque = btVector3(0, -0.5, 0);
        else
            torque = btVector3(0, 0.5, 0);
        
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;
        mRigidBody->applyTorque(correctedTorque);
    }

    //calculate velocity
    double decisionsPerSecond = _cyclesPerSecond / _cyclesPerDecision;
    double deceleration = (-(mCurrVel * mCurrVel) / (2*_frontRayDistance - 3)) / decisionsPerSecond;
    deceleration = deceleration < 0 ? deceleration : 0;

    mCurrVel += deceleration;
    if(mCurrVel < 0)
        mCurrVel = 0;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);
}

void CarAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 2);

    mAvoidanceMode = false;

    //double angularAcc = (_nnOutput[0] - 0.5) * (0.5 + (mMaxLinearVel - mCurrVel) / (mMaxLinearVel * 2));
    double angularAcc = _nnOutput[0] - 0.5;

    mRigidBody->applyTorque(btVector3(0, angularAcc/2, 0));

    double currAcc = _nnOutput[1] - 0.5;

    mCurrVel += currAcc * 10;
    if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;
    else if(mCurrVel < 0) mCurrVel = 0;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);
}

void CarAgent::tick(){
    btVector3 currAngVel = mRigidBody->getAngularVelocity();

    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }

    btVector3 currLinVel = mRigidBody->getLinearVelocity();

    mCurrVel = vector3(0, 0, 0).calcDistance(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));
}

vector3 CarAgent::getVelocity(){
    btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
    return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
}

btCollisionShape* CarAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void CarAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.7);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
    mRigidBody->setAngularFactor(btVector3(0, 1, 0));
    mRigidBody->setLinearFactor(btVector3(1, 0, 1));
}

btVector3 CarAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}