#ifndef STATICWORLDAGENT_H
#define STATICWORLDAGENT_H

#include "agent.h"

class StaticWorldAgent : public Agent
{
public:
    StaticWorldAgent(double _restitution, double _friction){
        mRestitution = _restitution;
        mFriction = _friction;
    }
    
    virtual ~StaticWorldAgent(){}

    virtual void update(const vector<double>& _nnOutput){
        //do not need to update static world objects
    }

    virtual void tick(){
        //no need to conform anything per tick
    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, true, true, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(mRestitution);
        mRigidBody->setFriction(mFriction);
        mRigidBody->setSleepingThresholds(0.f, 0.0f);
    }

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape){
        return btVector3(0, 0, 0);
    }

private:
    double mRestitution;
    double mFriction;

};

#endif