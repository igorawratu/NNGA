#ifndef AGENT_H
#define AGENT_H

#include "common.h"
#include "resourcemanager.h"

#include <vector>
#include <string>
#include <iostream>

#include <btBulletDynamicsCommon.h>

using namespace std;

class Agent
{
public:
    Agent(){}

    bool initialise(const string& _modelName, const vector3& _scale, const btQuaternion& _rotation, ResourceManager* _rm, const vector3& _position, const double& _mass){
        mModelName = _modelName;
        mScale = _scale;

        btCollisionShape* shape = getCollisionShape(_rm);
        if(!shape){
            cerr << "Error: unable to get collision shape for model " << _modelName << endl;
            return false;
        }

        btMotionState* ms = new btDefaultMotionState(btTransform(_rotation, btVector3(_position.x, _position.y, _position.z)));

        btVector3 inertia = calculateInertia(_mass, shape);
        
        btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

        mRigidBody = new btRigidBody(constructionInfo);

        setRigidbodyProperties();
    }

    virtual ~Agent(){
        delete mRigidBody->getCollisionShape();
        delete mRigidBody->getMotionState();
        delete mRigidBody;
    }

    virtual void update(const vector<double>& _nnOutput)=0;

    btRigidBody* getRigidBody(){
        return mRigidBody;
    }

    string getModelName(){
        return mModelName;
    }

    vector3 getScale(){
        return mScale;
    }

    virtual void tick()=0;

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm)=0;
    virtual void setRigidbodyProperties()=0;
    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape)=0;

protected:
    btRigidBody* mRigidBody;
    string mModelName;
    vector3 mScale;
    
};

#endif