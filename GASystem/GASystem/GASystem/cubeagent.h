#ifndef CUBEAGENT_H
#define CUBEAGENT_H

#include "agent.h"

class CubeAgent : public Agent
{
public:
    CubeAgent(vector3 _maxVel, vector3 _minVel);
    
    virtual ~CubeAgent();

    virtual void update(const vector<double>& _nnOutput);

    virtual vector3 getVelocity();

    void setPosition(vector3 _pos);

    virtual void tick();

    virtual void avoidCollisions(double _distanceLeft, double _distanceRight, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody);

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);

    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    vector3 mMaxVel;
    vector3 mMinVel;
};

#endif