#include "warrobotagent.h"

WarRobotAgent::WarRobotAgent(double _maxLinearVel, vector3 _maxVel, vector3 _minVel, double _maxAngularVel, uint _cooldownRate){
    mMaxVel = _maxVel;
    mMinVel = _minVel;
    mMaxAngularVel = _maxAngularVel;
    mCurrVel = 0;
    mCooldownRate = _cooldownRate;
    mCanShoot = 0;
    mMaxLinearVel = _maxLinearVel;
}

void WarRobotAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world){
    double left = getRayCollisionDistance(btVector3(100, 0, -100), _world);
    double right = getRayCollisionDistance(btVector3(100, 0, 100), _world);

    //calculate rotation
    /*if(left < _frontRayDistance && right < _frontRayDistance){
        //account for special case
    }
    else{*/
        btVector3 torque;

        if(right > left)
            torque = btVector3(0, -0.75, 0);
        else if(left > right)
            torque = btVector3(0, 0.75, 0);
        else if(left == right){
            int choose = generateRandInt();

            if(choose % 2 == 0)
                torque = btVector3(0, 0.75, 0);
            else torque = btVector3(0, -0.75, 0);
        }
        
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;
        mRigidBody->applyTorque(correctedTorque);
    //}

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

void WarRobotAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 3);

    /*double currAccX = _nnOutput[1] - 0.5;
    double currAccZ = _nnOutput[2] - 0.5;

    btVector3 vel = mRigidBody->getLinearVelocity();
    mRigidBody->setLinearVelocity(btVector3(vel.getX() + currAccX, 0, vel.getZ() + currAccZ));*/

    double currAcc = _nnOutput[1] - 0.5;

    mCurrVel += currAcc/* * 10*/;
    if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;
    else if(mCurrVel < 0) mCurrVel = 0;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);

    /*if(mCurrVel <= 1)
        mRigidBody->applyTorque(btVector3(0, (_nnOutput[0] - 0.5)/2, 0));
    else{
        mRigidBody->setAngularVelocity(btVector3(0, 0, 0));
    }*/
}

bool WarRobotAgent::shootRay(){
    if(mCanShoot == 0){
        mCanShoot = mCooldownRate;
        return true;
    }
    else return false;
}

void WarRobotAgent::tick(){
    btVector3 currAngVel = mRigidBody->getAngularVelocity();

    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }

    /*vector3 newVelocity(0, 0, 0);

    if(mRigidBody->getLinearVelocity().getX() > mMaxVel.x)
        newVelocity.x = mMaxVel.x;
    else if(mRigidBody->getLinearVelocity().getX() < mMinVel.x)
        newVelocity.x = mMinVel.x;
    else newVelocity.x = mRigidBody->getLinearVelocity().getX();

    if(mRigidBody->getLinearVelocity().getZ() > mMaxVel.z)
        newVelocity.z = mMaxVel.z;
    else if(mRigidBody->getLinearVelocity().getZ() < mMinVel.z)
        newVelocity.z = mMinVel.z;
    else newVelocity.z = mRigidBody->getLinearVelocity().getZ();

    mRigidBody->setLinearVelocity(btVector3(newVelocity.x, newVelocity.y, newVelocity.z));*/

    btVector3 currLinVel = mRigidBody->getLinearVelocity();

    mCurrVel = vector3(0, 0, 0).calcDistance(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));

    mCanShoot = mCanShoot == 0 ? 0 : mCanShoot - 1;
}

vector3 WarRobotAgent::getVelocity(){
    btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
    return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
}

btCollisionShape* WarRobotAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void WarRobotAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.7);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
    mRigidBody->setAngularFactor(btVector3(0, 1, 0));
    mRigidBody->setLinearFactor(btVector3(1, 0, 1));
}

btVector3 WarRobotAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}