#ifndef STARFIGHTERAGENT_H
#define STARFIGHTERAGENT_H

#include "agent.h"

class StarFighterAgent : public Agent
{
public:
    StarFighterAgent(double _maxLinearVel, double _maxAngularVel);

    virtual void update(const vector<double>& _nnOutput);

    virtual void tick();

    virtual vector3 getVelocity();

    virtual void avoidCollisions(double _distanceLeft, double _distanceRight, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody);


protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);

    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    double mMaxLinearVel;
    double mMaxAngularVel;
    double mCurrVel;
};

#endif