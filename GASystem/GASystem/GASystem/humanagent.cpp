#include "humanagent.h"

using namespace std;

HumanAgent::HumanAgent(double _maxLinearVel, double _maxAngularVel){
    mMaxLinearVel = _maxLinearVel;
    mMaxAngularVel = _maxAngularVel;
    mCurrVel = 0;
}

void HumanAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 4);

    mAvoidanceMode = false;

    double angularAcc = _nnOutput[0] - 0.5;

    mRigidBody->applyTorque(btVector3(0, angularAcc/2, 0));

    double currAcc = _nnOutput[1] - 0.5;

    mCurrVel += currAcc * 10;
    if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;
    else if(mCurrVel < 0) mCurrVel = 0;

    mCurrVel *= _nnOutput[3];

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);

    if(mAnimationLoop){
        if(mCurrVel == 0)
            setAnimationInfo("idle", true);
        else if(mCurrVel < 4)
            setAnimationInfo("walk", true);
        else setAnimationInfo("run", true);
    }
}

void HumanAgent::tick(){
    btVector3 currAngVel = mRigidBody->getAngularVelocity();

    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }

    btVector3 currLinVel = mRigidBody->getLinearVelocity();

    mCurrVel = vector3(0, 0, 0).calcDistance(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));
}

vector3 HumanAgent::getVelocity(){
    btVector3 vel = mRigidBody->getLinearVelocity();
    return vector3(vel.getX(), vel.getY(), vel.getZ());
}

void HumanAgent::avoidCollisions(double _distanceLeft, double _distanceRight, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    double left = _distanceLeft;
    double right = _distanceRight;
    double _frontRayDistance = left < right ? left : right;

    if(_frontRayDistance < 1.5)
        _frontRayDistance = 1.5001;

    if(mAnimationLoop){
        if(mCurrVel == 0)
            mAnimationName = "idle";
        else if(mCurrVel < 4)
            mAnimationName = "walk";
        else mAnimationName = "run";
    }
    
    //calculate rotation
    if(!mAvoidanceMode){
        if(left > right){
            mAvoidanceTorque = btVector3(0, -0.5, 0);
        }
        else{
            mAvoidanceTorque = btVector3(0, 0.5, 0);
        }
    }
    
    mAvoidanceMode = true;

    btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * mAvoidanceTorque;
    mRigidBody->applyTorque(correctedTorque);

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

btCollisionShape* HumanAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void HumanAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.5);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
    mRigidBody->setAngularFactor(btVector3(0, 1, 0));
    mRigidBody->setLinearFactor(btVector3(1, 0, 1));
}

btVector3 HumanAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}

void HumanAgent::setAnimationInfo(string _animationName, bool _loop){
    mAnimationName = _animationName;
    mAnimationLoop = _loop;
}