#include "sfobstaclefieldsimulation.h"

SFObstaclefieldSimulation::SFObstaclefieldSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
: SFSimulation(_rangefinderRadius, _numAgents, _numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, _seed){
}


SFObstaclefieldSimulation::SFObstaclefieldSimulation(const SFObstaclefieldSimulation& other) : SFSimulation(other.mRangefinderRadius, other.mAgents.size(), other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager, other.mSeed){
}


SFObstaclefieldSimulation::~SFObstaclefieldSimulation(){
}

Simulation* SFObstaclefieldSimulation::getNewCopy(){
    Simulation* sim = new SFObstaclefieldSimulation(*this);
    sim->initialise();

    return sim;
}

bool SFObstaclefieldSimulation::initialise(){
    if(mInitialised)
        return true;

    mGoalpoint = vector3(40, 0, 0);

    vector3 minDim(-50, -10, -10), maxDim(-30, 10, 10);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> disty(minDim.y, maxDim.y);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> geny(rng, disty);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new StarFighterAgent(10, 0.5);
        vector3 pos(genx(), geny(), genz());
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["goalpoint"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["goalpoint"]->initialise("sphere.mesh", vector3(mGoalRadius, mGoalRadius, mGoalRadius), btQuaternion(0, 0, 0, 1), mResourceManager, mGoalpoint, 0))
        return false;

    mWorldEntities["sfobstacle"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["sfobstacle"]->initialise("sfobstaclefield.mesh", vector3(20, 20, 20), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["sfobstacle"]->getRigidBody());

    mInitialised = true;
    
    return true;
}