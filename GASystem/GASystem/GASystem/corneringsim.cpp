#include "corneringsim.h"

CorneringSim::CorneringSim(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed, TeamSetup _setup) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, _setup){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderVals = 0;
    mRangefinderRadius = _rangefinderRadius;
    mAngularVelAcc = 0;

    for(uint k = 0; k < _numAgents; k++){
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
        mWaypointTracker[mAgents[k]] = 0;
    }
}   

CorneringSim::~CorneringSim(){

}

CorneringSim::CorneringSim(const CorneringSim& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager, other.mTeamSetup){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderVals = 0;
    mRangefinderRadius = other.mRangefinderRadius;
    mAngularVelAcc = 0;

    for(uint k = 0; k < other.mAgents.size(); k++){
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
        mWaypointTracker[mAgents[k]] = 0;
    }
}   

void CorneringSim::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        if(mTeamSetup == TeamSetup::HET){
            for(uint k = 0; k < mAgents.size(); k++)
                applyUpdateRules(mAgents[k], k);
        }
        else if(mTeamSetup == TeamSetup::QUARTHET){
            for(uint k = 0; k < mAgents.size(); k++)
                applyUpdateRules(mAgents[k], k / 2);
        }
        else if(mTeamSetup == TeamSetup::SEMIHET){
            for(uint k = 0; k < mAgents.size(); k++)
                applyUpdateRules(mAgents[k], k / 2);
        }
        else if(mTeamSetup == TeamSetup::HOM){
            for(uint k = 0; k < mAgents.size(); k++)
                applyUpdateRules(mAgents[k], 0);
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CorneringSim::fitness(){
    double finalFitness = 0;
    double maxCollisions = (mNumCycles / mCyclesPerDecision) * mAgents.size() * 9;

    map<string, vector3> pos;
    map<string, double> doubleAcc;
    map<string, long> intAcc;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["Collisions"] = mCollisions + mRangefinderVals;
    doubleAcc["WPFitnessWeight"] = 2;
    intAcc = mWaypointTracker;

    for(uint k = 0; k < mWaypoints.size(); k++)
        pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k];

    for(uint k = 0; k < mAgents.size(); k++){
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);
    }

    doubleAcc["LowerBound"] = 0;
    doubleAcc["UpperBound"] = (mAgents.size() * mNumCycles / mCyclesPerDecision) / 5;
    doubleAcc["Value"] = mAngularVelAcc;
    doubleAcc["EVWeight"] = 1;

    
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc);

    return finalFitness;
}

double CorneringSim::realFitness(){
    double finalFitness = 0;
    double maxCollisions = (mNumCycles / mCyclesPerDecision) * mAgents.size() * 9;

    map<string, vector3> pos;
    map<string, double> doubleAcc;
    map<string, long> intAcc;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["Collisions"] = mCollisions;
    doubleAcc["WPFitnessWeight"] = 2;
    intAcc = mWaypointTracker;

    for(uint k = 0; k < mWaypoints.size(); k++)
        pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k];

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    doubleAcc["LowerBound"] = 0;
    doubleAcc["UpperBound"] = (mAgents.size() * mNumCycles / mCyclesPerDecision) / 5;
    doubleAcc["Value"] = mAngularVelAcc;
    doubleAcc["EVWeight"] = 1;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc);


    return finalFitness;
}

Simulation* CorneringSim::getNewCopy(){
    Simulation* sim = new CorneringSim(*this);
    sim->initialise();

    return sim;
}

bool CorneringSim::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new WaypointFitness());
    mFitnessFunctions.push_back(new CollisionFitness());
    mFitnessFunctions.push_back(new ExpectedValueFitness());

    mWaypointTracker["ColFitnessWeight"] = 1;
    mWaypointTracker["NumAgents"] = mAgents.size();

    mWaypoints.push_back(vector3(-35, 0, -40));
    mWaypoints.push_back(vector3(5, 0, -30));
    mWaypoints.push_back(vector3(25, 0, 30));

    mWaypointTracker["NumWaypoints"] = mWaypoints.size();

    //set the vals
    vector3 minDim(-45, 0, 25), maxDim(-40, 0, 35);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new CarAgent(15, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), minDim.y, genz()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("corneringtrack.mesh", vector3(50, 5, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void CorneringSim::applyUpdateRules(string _agentName, uint _index){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    if(mWaypointTracker[_agentName] < mWaypoints.size()){
        if(vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()).calcDistance(mWaypoints[mWaypointTracker[_agentName]]) < 5)
            mWaypointTracker[_agentName]++;
    }

    map<uint, double> input;

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));

    double frontDist = d1 > d2 ? d2 : d1;

    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 0), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, 100), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, -100), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -100), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 100), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, -100), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 100), AGENT) / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[9] = agentVel.x;
    input[10] = agentVel.z;

    input[11] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].x;
    input[12] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].z;

    //agent position
    input[13] = trans.getOrigin().getX() / 50;
    input[14] = trans.getOrigin().getZ() / 50;

    double angVel = mWorldEntities[_agentName]->getAngularVelocity().y;
    input[15] = angVel;

    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(d2, d1, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{
        mWorldEntities[_agentName]->avoided();
        vector<double> output = mSolution->evaluateNeuralNetwork(_index, input);
        mWorldEntities[_agentName]->update(output);
    }

    if(mWaypointTracker[_agentName] < mWaypoints.size() && mCycleCounter > 10){
        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < mRangefinderRadius)
                mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
        
        mAngularVelAcc += fabs(angVel);

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
}

void CorneringSim::tick(){
    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]]->tick();

        if(mWaypointTracker[mAgents[k]] < mWaypoints.size() && mCycleCounter > 10){
            //gets collision data
            int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	        for (int i=0;i<numManifolds;i++){
		        btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
                if(contactManifold->getNumContacts() < 1)
                    continue;

		        const btCollisionObject* obA = contactManifold->getBody0();
		        const btCollisionObject* obB = contactManifold->getBody1();
                
                if((mWorldEntities[mAgents[k]]->getRigidBody() == obA || mWorldEntities[mAgents[k]]->getRigidBody() == obB))
                    mCollisions++;
            }
        }
    }

}

ESPParameters CorneringSim::getESPParams(string _nnFormatFile){
	ESPParameters params;
    params.populationSize = 50;
    params.maxGenerations = 999999;
    params.maxCompGenerations = 0;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = -1;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.1;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = params.populationSize/10;
    params.sampleEvaluationsPerChromosome = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;
    params.deltaCodeRadius = 0.1;

    return params;
}

StandardGAParameters CorneringSim::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 999999;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 50;
    params.fitnessEpsilonThreshold = -1;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.1;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = params.populationSize/10;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    return params;
}

CMAESParameters CorneringSim::getCMAESParameters(string _nnFormatFile){
    CMAESParameters params;

    params.maxGenerations = 999999;
    params.maxCompGenerations = 0;
    params.evalsPerCompChrom = 5;
    params.nnFormatFilename = _nnFormatFile;
    params.fitnessEpsilonThreshold = -1;
    params.deltaCodeRadius = 0.2;
    params.initStepsize = 0.2;

    return params;
}