#ifndef WARROBOTAGENT_H
#define WARROBOTAGENT_H

#include "agent.h"
#include "common.h"

class WarRobotAgent : public Agent
{
public:
    WarRobotAgent(double _maxLinearVel, vector3 _maxVel, vector3 _minVel, double _maxAngularVel, uint _cooldownRate){
        mMaxVel = _maxVel;
        mMinVel = _minVel;
        mMaxAngularVel = _maxAngularVel;
        mCurrVel = 0;
        mCooldownRate = _cooldownRate;
        mCanShoot = 0;
        mMaxLinearVel = _maxLinearVel;
    }

    virtual void update(const vector<double>& _nnOutput){
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

        if(mCurrVel <= 1)
            mRigidBody->applyTorque(btVector3(0, (_nnOutput[0] - 0.5)/2, 0));
        else{
            mRigidBody->setAngularVelocity(btVector3(0, 0, 0));
        }
    }

    bool shootRay(){
        if(mCanShoot == 0){
            mCanShoot = mCooldownRate;
            return true;
        }
        else return false;
    }

    virtual void tick(){
        btVector3 currAngVel = mRigidBody->getAngularVelocity();

        if(calcDistance(vector3(0, 0, 0), vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
            vector3 newAngVel = normalize(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()), mMaxAngularVel);
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

        mCurrVel = calcDistance(vector3(0, 0, 0), vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));

        mCanShoot = mCanShoot == 0 ? 0 : mCanShoot - 1;
    }

    virtual vector3 getVelocity(){
        btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
        return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
    }


protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(0.7);
        mRigidBody->setSleepingThresholds(0.f, 0.0f);
        mRigidBody->setAngularFactor(btVector3(0, 1, 0));
        mRigidBody->setLinearFactor(btVector3(1, 0, 1));
    }

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape){
        btVector3 inertia(0, 0, 0);
        _shape->calculateLocalInertia(_mass, inertia);

        return inertia;
    }

private:
    vector3 normalize(vector3 _vec, double _normVal){
        double mag = calcDistance(vector3(0, 0, 0), _vec);
        _vec.x = _vec.x / mag * _normVal;
        _vec.y = _vec.y / mag * _normVal;
        _vec.z = _vec.z / mag * _normVal;

        return _vec;
    }

    double calcDistance(vector3 _from, vector3 _to){
        double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

        return sqrt(x*x + y*y + z*z);
    }

private:
    vector3 mMaxVel, mMinVel;
    double mMaxLinearVel;
    double mMaxAngularVel;
    double mCurrVel;
    uint mCooldownRate, mCanShoot;
};

#endif