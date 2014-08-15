#ifndef POLEBALANCINGSIMULATION_H
#define POLEBALANCINGSIMULATION_H

#include <vector>
#include <math.h>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"

#define GRAVITY	9.81

#define MIN_X_POSITION	-2.5
#define MAX_X_POSITION	2.5

struct PoleCart{
  float pole_length;
  float cart_mass;
  float pole_mass;
  float cart_x_position;
  float cart_x_velocity;
  float max_angle;
  float min_angle;
  float pole_angle;
  float pole_angular_velocity;
};


class PoleBalancingSimulation : public Simulation
{
public:
    PoleBalancingSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    PoleBalancingSimulation(const PoleBalancingSimulation& other);
    virtual ~PoleBalancingSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        PoleBalancingSimulation* sim = (PoleBalancingSimulation*)world->getWorldUserInfo();
        sim->tick();
    }
    virtual vector<Line> getLines();
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);
    virtual CMAESParameters getCMAESParameters(string _nnFormatFile);

private:
    bool isCartFine();
    void iterateCart();
    
private:
    bool mComplete;
    PoleCart mCart;
    double mLastAppliedForce;
    int mSeed;
};

#endif