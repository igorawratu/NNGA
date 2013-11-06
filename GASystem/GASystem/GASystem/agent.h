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
    Agent();

    bool initialise(const string& _modelName, const vector3& _scale, const btQuaternion& _rotation, ResourceManager* _rm, const vector3& _position, const double& _mass);

    virtual ~Agent();

    virtual void update(const vector<double>& _nnOutput)=0;

    btRigidBody* getRigidBody();

    string getModelName();

    vector3 getScale();

    virtual vector3 getVelocity()=0;

    virtual void tick()=0;

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm)=0;
    virtual void setRigidbodyProperties()=0;
    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape)=0;

protected:
    btRigidBody* mRigidBody;
    string mModelName;
    vector3 mScale;
    btDynamicsWorld* mWorld;
};

#endif