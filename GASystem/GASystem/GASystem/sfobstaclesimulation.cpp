#include "sfobstaclesimulation.h"

SFObstacleSimulation::SFObstacleSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
: SFSimulation(_rangefinderRadius, _numAgents, _numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, _seed){
}


SFObstacleSimulation::SFObstacleSimulation(const SFObstacleSimulation& other) : SFSimulation(other.mRangefinderRadius, other.mAgents.size(), other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager, other.mSeed){
}


SFObstacleSimulation::~SFObstacleSimulation(){
}

Simulation* SFObstacleSimulation::getNewCopy(){
    Simulation* sim = new SFObstacleSimulation(*this);
    sim->initialise();

    return sim;
}

bool SFObstacleSimulation::initialise(){
    if(mInitialised)
        return true;

    mGoalpoint = vector3(80, 0, 0);
    mGoalRadius = 5;

    vector3 minDim(-100, -20, -10), maxDim(-80, 20, 10);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngy(mSeed * 2);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> disty(minDim.y, maxDim.y);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> geny(rngy, disty);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new StarFighterAgent(25, 0.5);
        vector3 pos(genx(), geny(), genz());
        if(!mWorldEntities[mAgents[k]]->initialise("starfighter.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["goalpoint"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["goalpoint"]->initialise("sphere.mesh", vector3(mGoalRadius, mGoalRadius, mGoalRadius), btQuaternion(0, 0, 0, 1), mResourceManager, mGoalpoint, 0, mSeed))
        return false;

    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("sfobstacle.mesh", vector3(20, 20, 20), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}