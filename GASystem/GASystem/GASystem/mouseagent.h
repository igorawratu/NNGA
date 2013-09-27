#ifndef MOUSEAGENT_H
#define MOUSEAGENT_H

#include "agent.h"

#include <iostream>

using namespace std;

enum RotationType{NOROTATION, RIGHT, LEFT};

class MouseAgent : public Agent
{
public:
    MouseAgent(double _maxLinearVel, double _maxAngularVel){
        mMaxLinearVel = _maxLinearVel;
        mMaxAngularVel = _maxAngularVel;
        mCurrVel = 0;
    }

    virtual void update(const vector<double>& _nnOutput){
        assert(_nnOutput.size() >= 3);
        
        mRigidBody->applyTorque(btVector3(0, (_nnOutput[0] - 0.5)/2, 0));

        double currAcc = _nnOutput[1];
        mCurrVel += currAcc * 2;
        if(mCurrVel > mMaxLinearVel)
            mCurrVel = mMaxLinearVel;

        mCurrVel *= _nnOutput[2];

        btVector3 relativeVel = btVector3(mCurrVel, 0, 0);

        btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
        btVector3 correctedVel = rot * relativeVel;
        correctedVel.setY(0);

        mRigidBody->setLinearVelocity(correctedVel);
    }

    virtual vector3 getVelocity(){
        btVector3 correctedVel = mRigidBody->getWorldTransform().getBasis() * btVector3(mCurrVel, 0, 0);
        return vector3(correctedVel.getX(), correctedVel.getY(), correctedVel.getZ());
    }

    virtual void tick(){
        btVector3 currAngVel = mRigidBody->getAngularVelocity();

        if(calcDistance(vector3(0, 0, 0), vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ())) > mMaxAngularVel){
            vector3 newAngVel = normalize(vector3(currAngVel.getX(), currAngVel.getY(), currAngVel.getZ()), mMaxAngularVel);
            mRigidBody->setAngularVelocity(btVector3(newAngVel.x, newAngVel.y, newAngVel.z));
        }

        btVector3 currLinVel = mRigidBody->getLinearVelocity();

        mCurrVel = calcDistance(vector3(0, 0, 0), vector3(currLinVel.getX(), currLinVel.getY(), currLinVel.getZ()));
    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(0.1);
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
    double mMaxLinearVel;
    double mMaxAngularVel;
    double mCurrVel;
};

#endif