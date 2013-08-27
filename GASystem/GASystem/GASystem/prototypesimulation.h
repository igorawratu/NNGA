#ifndef PROTOTYPESIMULATION_H
#define PROTOTYPESIMULATION_H

#include "simulation.h"
#include "common.h"

class PrototypeSimulation : public Simulation
{
public:
    PrototypeSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution);
    virtual ~PrototypeSimulation();
    virtual void iterate();
    virtual double fitness(vector<Fitness*> _fit);
    virtual Simulation* getNewCopy();
    virtual bool initialise(ResourceManager* _rm);
    void conformVelocities();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        PrototypeSimulation* sim = (PrototypeSimulation*)world->getWorldUserInfo();
        sim->conformVelocities();
    }

private:
    bool createObject(string _meshname, string _entityName, vector3 _scale, vector3 _position, float _mass, ResourceManager* _rm);
    void getRayCollisionDistances(map<uint, double>& _collisionDistances, const btVector3& _agentPosition);
    void applyUpdateRules(string _agentName);
};

#endif