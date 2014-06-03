#include "evacuationsimulation.h"

EvacuationSimulation::EvacuationSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(EvacuationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;
    mAngularVelAcc = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

EvacuationSimulation::~EvacuationSimulation(){
    
}

EvacuationSimulation::EvacuationSimulation(const EvacuationSimulation& other){
    mWorld->setInternalTickCallback(EvacuationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;
    mAngularVelAcc = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

void EvacuationSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    for(int k = 0; k < mAgents.size(); k++)
        applyUpdateRules(mAgents[k], 0);

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double EvacuationSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["FLFitnessWeight"] = 1;
    intAcc["Positive"] = 0;
    pos["LineP1"] = mExit.p1;
    pos["LineP2"] = mExit.p2;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

double EvacuationSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["FLFitnessWeight"] = 1;
    intAcc["Positive"] = 0;
    pos["LineP1"] = mExit.p1;
    pos["LineP2"] = mExit.p2;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

Simulation* EvacuationSimulation::getNewCopy(){
    Simulation* sim = new EvacuationSimulation(*this);
    sim->initialise();
    
    return sim;
}

bool EvacuationSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());

    //change
    mFinishLine.p1 = vector3(-10, 0, -25);
    mFinishLine.p2 = vector3(10, 0, -25);

    mLines.push_back(mFinishLine);

    //set the vals
    vector3 minDim(-20, 0, 25), maxDim(25, 0, 40);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new HumanAgent(5, 10, 0, 1);
        vector3 pos(genx(), 0, genz());
        if(!mWorldEntities[mAgents[k]]->initialise("human.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01, mSeed))
            return false;
        mWorldEntities[mAgents[k]]->setAnimationInfo("idle", true);
        
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }


    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("evacuationenvironment.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, -1, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void EvacuationSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]]->tick();

        /*btTransform trans;
        mWorldEntities[mAgents[k]]->getRigidBody()->getMotionState()->getWorldTransform(trans);

        if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0 && mCycleCounter > 10){
            int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	        for (int i=0;i<numManifolds;i++)
	        {
		        btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
                if(contactManifold->getNumContacts() < 1)
                    continue;

		        const btCollisionObject* obA = contactManifold->getBody0();
		        const btCollisionObject* obB = contactManifold->getBody1();
                
                if((mWorldEntities[mAgents[k]]->getRigidBody() == obA || mWorldEntities[mAgents[k]]->getRigidBody() == obB))
                    mCollisions++;
            }
        }*/
    }
}

void EvacuationSimulation::applyUpdateRules(string _agentName){
    //do nothing if agent still busy with non looping animation
    if(!mWorldEntities[_agentName]->getAnimationLoop())
        return;

    //check for pushing/staggering here

    //query ANN
    map<uint, double> input;
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 105), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(100, 0, 75), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(100, 0, 45), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(100, 0, 15), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -15), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(100, 0, -45), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(100, 0, -75), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, -105), AGENT) / 50;

    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[11] = mExit.p1.x / 50;
    input[12] = mExit.p1.z / 50;
    input[13] = mExit.p2.x / 50;
    input[14] = mExit.p2.z / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[15] = agentVel.x;
    input[16] = agentVel.z;

    double angVel = mWorldEntities[_agentName]->getAngularVelocity().y;
    input[17] = angVel;

    vector<double> output = mSolution->evaluateNeuralNetwork(_groupNum, input);

    mWorldEntities[_agentName]->update(output);

    if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0 && mCycleCounter > 10){
        mAngularVelAcc += fabs(angVel);

        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < mRangefinderRadius)
                mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
    }
}
