#include "bridgesimulation.h"

BridgeSimulation::BridgeSimulation(double _rangefinderRadius, uint _numAgents, AgentType _agentType, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(BridgeSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mAgentType = _agentType;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

BridgeSimulation::BridgeSimulation(const BridgeSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(BridgeSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mAgentType = other.mAgentType;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

BridgeSimulation::~BridgeSimulation(){}

void BridgeSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double BridgeSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["Collisions"] = mRangefinderVals + mCollisions; 
    dblAcc["FLFitnessWeight"] = 1;
    dblAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : 1000;
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

Simulation* BridgeSimulation::getNewCopy(){
    Simulation* newsim = new BridgeSimulation(*this);
    newsim->initialise();

    return newsim;
}

void BridgeSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();

}

double BridgeSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["Collisions"] = mCollisions; 
    dblAcc["FLFitnessWeight"] = 1;
    dblAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : 1000;
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

bool BridgeSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mFinishLine.p1 = vector3(-10, 0, -25);
    mFinishLine.p2 = vector3(10, 0, -25);

    //set the vals
    vector3 minDim(-20, -7.1, 25), maxDim(25, -7, 40);

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

    if(mAgentType == CAR){
        for(uint k = 0; k < mAgents.size(); k++){
            mWorldEntities[mAgents[k]] = new CarAgent(10, 0.5);
            vector3 pos(genx(), geny(), genz());
            if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01, mSeed))
                return false;
            mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
        }
    }
    else if(mAgentType == MOUSE){
        for(uint k = 0; k < mAgents.size(); k++){
            mWorldEntities[mAgents[k]] = new MouseAgent(10, 1);
            if(!mWorldEntities[mAgents[k]]->initialise("mouse.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), geny(), genz()), 0.01, mSeed))
                return false;
            mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
        }
    }
    else{
        cerr << "Error: unrecognised agent type" << endl;
        return false;
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("newbridgewall.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, -1, 0), 0, mSeed))
        return false;
    mWorldEnv->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void BridgeSimulation::applyUpdateRules(string _agentName){
    map<uint, double> input;
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    double frontDist = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT);
    if(frontDist < 6)
        mWorldEntities[_agentName]->avoidCollisions(frontDist, mCyclesPerSecond, mCyclesPerDecision, mWorldEnv);
    else{
        //rangefinders
        if(mAgentType == CAR){
            input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) / 50;
            input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 0), AGENT) / 50;
            input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, 100), AGENT) / 50;
            input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, -100), AGENT) / 50;
            input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -100), AGENT) / 50;
            input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 100), AGENT) / 50;
            input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, -100), AGENT) / 50;
            input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 100), AGENT) / 50;
            frontVal = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT);
        }
        else if (mAgentType == MOUSE){
            input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 105), AGENT) / 50;
            input[2] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 75), AGENT) / 50;
            input[3] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 45), AGENT) / 50;
            input[4] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 15), AGENT) / 50;
            input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -15), AGENT) / 50;
            input[6] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -45), AGENT) / 50;
            input[7] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -75), AGENT) / 50;
            input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -105), AGENT) / 50;
            frontVal = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) > 5 ? 1 : 0;
        }
        else{
            cerr << "Error: unidentified agent type" << endl;
            return;
        }

        //agent position
        input[9] = trans.getOrigin().getX() / 50;
        input[10] = trans.getOrigin().getZ() / 50;
        
        //goal line
        input[11] = mFinishLine.p1.x / 50;
        input[12] = mFinishLine.p1.z / 50;
        input[13] = mFinishLine.p2.x / 50;
        input[14] = mFinishLine.p2.z / 50;

        vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
        input[15] = agentVel.x;
        input[16] = agentVel.z;

        vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
        output.push_back(frontVal);

        mWorldEntities[_agentName]->update(output);
    }


    if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0){
        if(frontDist >= 5){
            for(uint k = 1; k <= 8; k++)
                if(input[k] * 50 < mRangefinderRadius)
                    mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
        }
        
        //gets collision data
        int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	    for (int i=0;i<numManifolds;i++)
	    {
		    btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
            if(contactManifold->getNumContacts() < 1)
                continue;

		    const btCollisionObject* obA = contactManifold->getBody0();
		    const btCollisionObject* obB = contactManifold->getBody1();
            
            if((mWorldEntities[_agentName]->getRigidBody() == obA || mWorldEntities[_agentName]->getRigidBody() == obB)){
                mCollisions++;
                //cout << mCollisions << endl;
            }
        }
    }
}

vector<Line> BridgeSimulation::getLines(){
    vector<Line> lines;
    lines.push_back(mFinishLine);

    return lines;
}