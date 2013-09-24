#ifndef WARROBOTAGENT_H
#define WARROBOTAGENT_H

#include "agent.h"
#include <iostream>
#include <map>

using namespace std;

class WarRobotAgent : public Agent
{
public:
    WarRobotAgent(double _maxLinearVel, double _maxAngularVel, double _shootCooldown, double _timePerTick, map<Agent*, btCollisionWorld::ClosestRayResultCallback*>* _shootmap){
        mMaxLinearVel = _maxLinearVel;
        mMaxAngularVel = _maxAngularVel;
        mShootCooldown = _shootCooldown;
        mTimePerTick = _timePerTick;
        mTimeUntilNextShot = 0;
        mShootmap = _shootmap;
    }

    virtual ~WarRobotAgent();

    virtual void update(const vector<double>& _nnOutput){
        assert(_nnOutput.size() >= 3);

        //shoot
        if(mTimeUntilNextShot == 0 && _nnOutput[0] > 0.5){
            btVector3 relShootDir = btVector3(500, 0, 0);
            btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
            btVector3 corShootDir = rot * relShootDir;

            btCollisionWorld::ClosestRayResultCallback* shootRay = new btCollisionWorld::ClosestRayResultCallback(mRigidBody->getWorldTransform().getOrigin(), corShootDir);
            mWorld->rayTest(mRigidBody->getWorldTransform().getOrigin(), corShootDir, *shootRay);

            (*mShootmap)[this] = shootRay;

            mTimeUntilNextShot = mShootCooldown;
        }

        //rotation
        mRigidBody->applyTorque(btVector3(0, _nnOutput[1]/10, 0));

        //acceleration
        btVector3 relativeForce = btVector3(_nnOutput[2], 0, 0);
        btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
        btVector3 correctedForce = rot * relativeForce;
        mRigidBody->applyCentralForce(correctedForce);
    }

    virtual void tick(){
        btVector3 currAngVel = mRigidBody->getAngularVelocity();

        if(calcDistance(vector3(0, 0, 0), vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
            vector3 newAngVel = normalize(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()), mMaxAngularVel);
            mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
        }

        btVector3 currLinVel = mRigidBody->getLinearVelocity();

        if(calcDistance(vector3(0, 0, 0), vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ())) > mMaxLinearVel){
            vector3 newLinVel = normalize(vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()), mMaxLinearVel);
            mRigidBody->setLinearVelocity(btVector3(newLinVel.x, newLinVel.y, newLinVel.z));
        }

        if(mTimeUntilNextShot > 0)
            mTimeUntilNextShot = mTimeUntilNextShot - mTimePerTick < 0 ? 0 : mTimeUntilNextShot - mTimePerTick;
    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(0.1);
        mRigidBody->setSleepingThresholds(0.f, 0.0f);
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
    double mMaxLinearVel, mMaxAngularVel, mShootCooldown, mTimePerTick, mTimeUntilNextShot;
    map<Agent*, btCollisionWorld::ClosestRayResultCallback*>* mShootmap;
};

#endif