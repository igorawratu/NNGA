#include "formationsimulation.h"

FormationSimulation::FormationSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(FormationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < 20; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

FormationSimulation::FormationSimulation(const FormationSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(FormationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

FormationSimulation::~FormationSimulation(){}

void FormationSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double FormationSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mRangefinderVals + mCollisions; 
    doubleAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 1;
    doubleAcc["GoalRadius"] = mCircleRadius;
    pos["GoalPoint"] = mCircleCenter;

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc) : 1000; //change this amount

    return finalFitness;
}

Simulation* FormationSimulation::getNewCopy(){
    Simulation* newsim = new FormationSimulation(*this);
    newsim->initialise();

    return newsim;
}

void FormationSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();

}

double FormationSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mRangefinderVals + mCollisions; 
    doubleAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 1;
    doubleAcc["GoalRadius"] = mCircleRadius;
    pos["GoalPoint"] = mCircleCenter;

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc) : 1000; //change this amount

    return finalFitness;
}

bool FormationSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new GoalPointFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mCircleCenter = vector3(30, 0, 0);
    mCircleRadius = 10;

    btQuaternion rot(0, 0, 0, 1);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new CarAgent(10, 0.5);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(k % 5, 0, k/5), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("environmentbox.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorldEnv->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void FormationSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    map<uint, double> input;

    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 0), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, 100), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, -100), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -100), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 100), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, -100), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 100), AGENT) / 50;
    frontVal = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT);

    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[11] = mCircleCenter.x / 50;
    input[12] = mCircleCenter.z / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[13] = agentVel.x;
    input[14] = agentVel.z;

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
    output.push_back(frontVal);

    mWorldEntities[_agentName]->update(output);

    for(uint k = 1; k <= 8; k++)
        if(input[k] * 50 < mRangefinderRadius)
            mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
    
    //gets collision data
    int numManifolds = mWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
	    btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
        if(contactManifold->getNumContacts() < 1)
            continue;

	    const btCollisionObject* obA = contactManifold->getBody0();
	    const btCollisionObject* obB = contactManifold->getBody1();
        
        if((mWorldEntities[_agentName]->getRigidBody() == obA || mWorldEntities[_agentName]->getRigidBody() == obB))
            mCollisions++;
    }
}