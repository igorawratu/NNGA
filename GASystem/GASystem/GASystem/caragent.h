#ifndef CARAGENT_H
#define CARAGENT_H

#include "agent.h"

class CarAgent : public Agent
{
public:
    CarAgent(double _maxLinearVel, double _maxAngularVel){
        mMaxLinearVel = _maxLinearVel;
        mMaxAngularVel = _maxAngularVel;
    }

    virtual void update(const vector<double>& _nnOutput){
        assert(_nnOutput.size() >= 2);

        mRigidBody->applyTorque(btVector3(0, _nnOutput[0]/10, 0));

        double currAcc = _nnOutput[1] - 0.3;

        btVector3 relativeForce = btVector3(currAcc, 0, 0);
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
    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(0.7);
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
    double mMaxLinearVel;
    double mMaxAngularVel;
};

#endif