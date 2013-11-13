#ifndef WARROBOTAGENT_H
#define WARROBOTAGENT_H

#include "agent.h"
#include "common.h"

class WarRobotAgent : public Agent
{
public:
    WarRobotAgent(double _maxLinearVel, vector3 _maxVel, vector3 _minVel, double _maxAngularVel, uint _cooldownRate);

    virtual void update(const vector<double>& _nnOutput);

    bool shootRay();

    virtual void tick();

    virtual vector3 getVelocity();

    virtual void avoidCollisions(double _frontRayDistance, uint _cyclesPerSecond, uint _cyclesPerDecision, btDiscreteDynamicsWorld* _world);

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm);
    virtual void setRigidbodyProperties();

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape);

private:
    vector3 mMaxVel, mMinVel;
    double mMaxLinearVel;
    double mMaxAngularVel;
    double mCurrVel;
    uint mCooldownRate, mCanShoot;
};

#endif