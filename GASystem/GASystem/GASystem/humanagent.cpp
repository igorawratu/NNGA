#include "humanagent.h"

using namespace std;

HumanAgent::HumanAgent(double _walkVelocity, double _runVelocity, double _staggerVelocity, double _maxAngularVelocity){
    mWalkVelocity = _walkVelocity;
    mRunVelocity = _runVelocity;
    mStaggerVelocity = _staggerVelocity;
    mMaxAngularVel = _maxAngularVelocity;
    mCurrState = IDLE;
}

void HumanAgent::update(const vector<double>& _nnOutput){
    assert(_nnOutput.size() >= 2);

    double angularAcc = _nnOutput[0] - 0.5;

    mRigidBody->applyTorque(btVector3(0, angularAcc/2, 0));

    double currAcc = _nnOutput[1] - 0.5;
    if(_nnOutput[1] < 0.3)
        setAnimationInfo("idle", true);
    else if(_nnOutput[1] >= 0.3 && _nnOutput[1] < 0.7)
        setAnimationInfo("walk", true);
    else setAnimationInfo("run", true);
}

void HumanAgent::setState(HumanState _state){
    mCurrState = _state;
}

HumanState HumanAgent::getState(){
    return mCurrState;
}

void HumanAgent::tick(){
    double vel;

    switch(mCurrState){
        case IDLE: vel = 0;
            break;
        case WALK: vel = mWalkVelocity;
            break;
        case RUN: vel = mRunVelocity;
            break;
        case STAGGER: vel = mStaggerVelocity;
            break;
        case PUSH: vel = 0;
            break;
        default: vel = 0;
            break;
    }

    btVector3 relativeVel(vel, 0, 0);

    btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
    btVector3 correctedVel = rot * relativeVel;
    correctedVel.setY(0);

    mRigidBody->setLinearVelocity(correctedVel);

    btVector3 currAngVel = mRigidBody->getAngularVelocity();
    if(vector3(0, 0, 0).calcDistance(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
        vector3 newAngVel = vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()).normalize() * mMaxAngularVel;
        mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
    }
}

vector3 HumanAgent::getVelocity(){
    btVector3 vel = mRigidBody->getLinearVelocity();
    return vector3(vel.getX(), vel.getY(), vel.getZ());
}

void HumanAgent::avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    //try first with no ca
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
    if(_animationName == "idle")
        mCurrState = IDLE;
    else if(_animationName == "walk")
        mCurrState = WALK;
    else if(_animationName == "run")
        mCurrState = RUN;
    else if(_animationName == "stagger")
        mCurrState = STAGGER;
    else if(_animationName == "shove")
        mCurrState = PUSH;

    mAnimationName = _animationName;
    mAnimationLoop = _loop;
}