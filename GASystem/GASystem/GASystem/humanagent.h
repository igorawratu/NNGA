#ifndef HUMANAGENT_H
#define HUMANAGENT_H

#include "agent.h"
#include <iostream>

enum HumanState{IDLE, WALK, RUN, PUSH, STAGGER_BACK, STAGGER_FORWARD, AVOIDING};

class HumanAgent : public Agent
{
public:
    HumanAgent(double _maxLinearVel, double _maxAngularVel);

    virtual void update(const vector<double>& _nnOutput);

    void setState(HumanState _state);

    HumanState getState();

    virtual void tick();

    virtual vector3 getVelocity();

    virtual void avoidCollisions(double _distanceLeft, double _distanceRight, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody);

    virtual void setAnimationInfo(string _animationName, bool _loop);

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);

    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    double mMaxLinearVel;
    double mMaxAngularVel;
    double mCurrVel;
    btVector3 mAvoidanceTorque;
};

#endif