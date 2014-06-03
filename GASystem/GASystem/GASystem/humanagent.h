#ifndef HUMANAGENT_H
#define HUMANAGENT_H

#include "agent.h"

enum HumanState{IDLE, WALK, RUN, PUSH, STAGGER};

class HumanAgent : public Agent
{
public:
    HumanAgent(double _walkVelocity, double _runVelocity, double _staggerVelocity, double _maxAngularVelocity);

    virtual void update(const vector<double>& _nnOutput);

    void setState(HumanState _state);

    HumanState getState();

    virtual void tick();

    virtual vector3 getVelocity();

    virtual void avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody);

    virtual void setAnimationInfo(string _animationName, bool _loop)

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);

    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    double mMaxAngularVel, mWalkVelocity, mRunVelocity, mStaggerVelocity;
    HumanState mCurrState;
};

#endif