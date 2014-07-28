#ifndef MOUSEESCAPESIMULATION_H
#define MOUSEESCAPESIMULATION_H

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "warrobotagent.h"
#include "staticworldagent.h"
#include "finishlinefitness.h"
#include "collisionfitness.h"
#include "winnerfitness.h"
#include "expectedvaluefitness.h"
#include "mouseagent.h"

#include <iostream>
#include <vector>

class MouseEscapeSimulation : public Simulation
{
public:
    MouseEscapeSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~MouseEscapeSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    virtual double realFitness();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        MouseEscapeSimulation* sim = (MouseEscapeSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

    virtual vector<string> getRemoveList(){
        return mObjectsToRemove;
    }

    virtual vector<Line> getLines(){
        vector<Line> linesToRender = mRaysShot;
        linesToRender.push_back(mFinishLine);

        return linesToRender;
    }

    virtual vector<CompetitiveFitness> competitiveFitness();
	virtual ESPParameters getESPParams(string _nnFormatFile);
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile);

private:
    void applyUpdateRules(string _agentName, uint _groupNum);
    void checkRayObject(int _groupNum, const btCollisionObject* _obj, int& _team, string& _entityName);
    bool crossed(string _agentName);
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.x)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }

private:
    vector<string> mMouseAgents, mRobotAgents; 
    long mCollisions;
    double mRangefinderVals, mRangefinderRadius;
    int mSeed;
    double mVelocityAcc;
    vector<string> mObjectsToRemove;
    vector<string> mCrossed;
    vector<Line> mRaysShot;
    Line mFinishLine;
};

#endif