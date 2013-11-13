#include "mousescattersimulation.h"

MouseScatterSimulation::MouseScatterSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(MouseScatterSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

MouseScatterSimulation::MouseScatterSimulation(const MouseScatterSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(MouseScatterSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

MouseScatterSimulation::~MouseScatterSimulation(){}

void MouseScatterSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double MouseScatterSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["Collisions"] = mRangefinderVals + mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    dblAcc["EVFitnessWeight"] = 1;
    dblAcc["LowerBound"] = mAgents.size() * mNumCycles * 5;
    dblAcc["UpperBound"] = mAgents.size() * mNumCycles * 10;
    double distance = 0;
    for(uint k = 0; k < mAgents.size(); ++k)
        distance += getPositionInfo(mAgents[k]).calcDistance(mCenterPoint);
    dblAcc["Value"] = distance;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : mAgents.size() * mNumCycles * 5;
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

Simulation* MouseScatterSimulation::getNewCopy(){
    Simulation* newsim = new MouseScatterSimulation(*this);
    newsim->initialise();

    return newsim;
}

void MouseScatterSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}

double MouseScatterSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["Collisions"] = mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    dblAcc["EVFitnessWeight"] = 1;
    dblAcc["LowerBound"] = mAgents.size() * mNumCycles * 5;
    dblAcc["UpperBound"] = mAgents.size() * mNumCycles * 10;
    double distance = 0;
    for(uint k = 0; k < mAgents.size(); ++k)
        distance += getPositionInfo(mAgents[k]).calcDistance(mCenterPoint);
    dblAcc["Value"] = distance;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : mAgents.size() * mNumCycles * 5;
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

bool MouseScatterSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new ExpectedValueFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mCenterPoint = vector3(0, 0, 0);

    //set the vals
    vector3 minDim(-35, 0, -35), maxDim(35, 0, 35);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> disty(minDim.y, maxDim.y);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> geny(rng, disty);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);


    for(uint k = 0; k < mAgents.size(); k++){
        //set rotation to have an orientation facing away from centerpoint
        btQuaternion rot(0, 0, 0, 1);

        vector3 currAgentPos;
        currAgentPos.x = genx();
        currAgentPos.y = 0;
        currAgentPos.z = genz();

        vector3 directionVec = vector3(currAgentPos.x - mCenterPoint.x, currAgentPos.y - mCenterPoint.y, currAgentPos.z - mCenterPoint.z);
        vector3 originalVec = vector3(1, 0, 0);

        double dot = directionVec.x * originalVec.x + directionVec.y * originalVec.y + directionVec.z * originalVec.z;
        double mag = directionVec.calcDistance(vector3(0, 0, 0)) * originalVec.calcDistance(vector3(0, 0, 0));

        double c = dot/mag;

        rot.setEuler(acos(c), 0, 0);

        mWorldEntities[mAgents[k]] = new MouseAgent(10, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("mouse.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), 0, genz()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("stadium.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorldEnv->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void MouseScatterSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    map<uint, double> input;
    //rangefinders
    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 105), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 75), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 45), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 15), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -15), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -45), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -75), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -105), AGENT) / 50;
    frontVal = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) > 3 ? 1 : 0;

    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[11] = mCenterPoint.x / 50;
    input[12] = mCenterPoint.z / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[13] = agentVel.x / 10;
    input[14] = agentVel.z / 10;

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
