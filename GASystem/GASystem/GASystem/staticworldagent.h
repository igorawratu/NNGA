#ifndef STATICWORLDAGENT_H
#define STATICWORLDAGENT_H

#include "agent.h"

class StaticWorldAgent : public Agent
{
public:
    StaticWorldAgent(double _restitution, double _friction);
    
    virtual ~StaticWorldAgent();

    virtual void update(const vector<double>& _nnOutput);

    virtual void tick();

    virtual vector3 getVelocity();

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);
    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    double mRestitution;
    double mFriction;

};

#endif