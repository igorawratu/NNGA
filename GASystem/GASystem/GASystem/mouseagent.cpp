#include "mouseagent.h"

MouseAgent::MouseAgent(double _maxLinearVel, double _maxAngularVel){
    mMaxLinearVel = _maxLinearVel;
    mMaxAngularVel = _maxAngularVel;
    mCurrVel = 0;
}

void MouseAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world){
    double left = getRayCollisionDistance(btVector3(100, 0, -100), _world);
    double right = getRayCollisionDistance(btVector3(100, 0, 100), _world);

    //calculate rotation
    if(left < _frontRayDistance && right < _frontRayDistance){
        //account for special case
    }
    else{
        btVector3 torque;

        if(right > left)
            torque = btVector3(0, -1, 0);
        else if(left > right)
            torque = btVector3(0, 1, 0);
        else if(left == right){
            int choose = generateRandInt();

            if(choose % 2 == 0)
                torque = btVector3(0, 1, 0);
            else torque = btVector3(0, -1, 0);
        }
        
        btVector3 correctedTorque = mRigidBody->getWorldTransform().getBasis() * torque;
        mRigidBody->applyTorque(correctedTorque);
    }

    //calculate velocity
    double decisionsPerSecond = _cyclesPerSecond / _cyclesPerDecision;
    double deceleration = (-(mCurrVel * mCurrVel) / (2*_frontRayDistance - 3)) / decisionsPerSecond;

    mCurrVel += deceleration;
    cout << deceleration << endl;
    if(mCurrVel < 0)
        mCurrVel = 0;
    else if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);
}

void MouseAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 3);
    
    mRigidBody->applyTorque(btVector3(0, (_nnOutput[0] - 0.5)/2, 0));

    double currAcc = _nnOutput[1];
    mCurrVel += currAcc * 5;
    if(mCurrVel > mMaxLinearVel)
        mCurrVel = mMaxLinearVel;
    else if(mCurrVel < 0)
        mCurrVel = 0;

    mCurrVel *= _nnOutput[2];

    btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);
}

vector3 MouseAgent::getVelocity(){
    btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
    return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
}

void MouseAgent::tick(){
    btVector3 currAngVel = mRigidBody->getAngularVelocity();

    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }

    btVector3 currLinVel = mRigidBody->getLinearVelocity();

    mCurrVel = vector3(0, 0, 0).calcDistance(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));
}

btCollisionShape* MouseAgent::getCollisionShape(ResourceManager* _rm){
    return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
}

void MouseAgent::setRigidbodyProperties(){
    mRigidBody->setRestitution(0.1);
    mRigidBody->setSleepingThresholds(0.f, 0.0f);
    mRigidBody->setAngularFactor(btVector3(0, 1, 0));
    mRigidBody->setLinearFactor(btVector3(1, 0, 1));
}

btVector3 MouseAgent::calculateInertia(double _mass, btCollisionShape* _shape){
    btVector3 inertia(0, 0, 0);
    _shape->calculateLocalInertia(_mass, inertia);

    return inertia;
}