#ifndef MOUSEAGENT_H
#define MOUSEAGENT_H

#include "agent.h"

#include <iostream>

using namespace std;

enum RotationType{NOROTATION, RIGHT, LEFT};

class MouseAgent : public Agent
{
public:
    MouseAgent(double _maxLinearVel, double _maxAngularVel);

    virtual void update(const vector<double>& _nnOutput);

    virtual vector3 getVelocity();

    virtual void tick();

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