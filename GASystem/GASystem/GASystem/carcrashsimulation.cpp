#include "carcrashsimulation.h"

CarCrashSimulation::CarCrashSimulation(double _rangefinderRadius, uint _agentsPerSide, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed, TeamSetup _setup)
 : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, _setup){
    mWorld->setInternalTickCallback(CarCrashSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < _agentsPerSide; k++)
        mGroupOneAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    for(uint k = _agentsPerSide; k < _agentsPerSide * 2; k++)
        mGroupTwoAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

CarCrashSimulation::~CarCrashSimulation(){

}

void CarCrashSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(uint k = 0; k < mGroupOneAgents.size(); k++)
            applyUpdateRules(mGroupOneAgents[k], 0);
        for(uint k = 0; k < mGroupTwoAgents.size(); k++)
            applyUpdateRules(mGroupTwoAgents[k], 1);

        for(int k = 0; k < mGroupOneAgents.size(); k++){
            if(mTeamSetup == TeamSetup::HET){
                applyUpdateRules(mGroupOneAgents[k], k);
            }
            else if(mTeamSetup == TeamSetup::QUARTHET){
                applyUpdateRules(mGroupOneAgents[k], k / 5);
            }
            else if(mTeamSetup == TeamSetup::SEMIHET){
                applyUpdateRules(mGroupOneAgents[k], 0);
            }
            else if(mTeamSetup == TeamSetup::HOM){
                applyUpdateRules(mGroupOneAgents[k], 0);
            }
        }

        for(int k = 0; k < mGroupTwoAgents.size(); k++){
            if(mTeamSetup == TeamSetup::HET){
                applyUpdateRules(mGroupTwoAgents[k], k + mGroupOneAgents.size());
            }
            else if(mTeamSetup == TeamSetup::QUARTHET){
                applyUpdateRules(mGroupTwoAgents[k], (k / 5) + mGroupOneAgents.size() / 5);
            }
            else if(mTeamSetup == TeamSetup::SEMIHET){
                applyUpdateRules(mGroupTwoAgents[k], 1);
            }
            else if(mTeamSetup == TeamSetup::HOM){
                applyUpdateRules(mGroupTwoAgents[k], 0);
            }
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CarCrashSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mRangefinderVals + mCollisions; 
    doubleAcc["FLFitnessWeight"] = 5;
    doubleAcc["ColFitnessWeight"] = 1;

    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        pos[mGroupOneAgents[k]] = getPositionInfo(mGroupOneAgents[k]);
    intAcc["Positive"] = 1;
    pos["LineP1"] = mGroupOneFinish.p1;
    pos["LineP2"] = mGroupOneFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);

    pos.clear();

    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        pos[mGroupTwoAgents[k]] = getPositionInfo(mGroupTwoAgents[k]);
    intAcc["Positive"] = 0;
    pos["LineP1"] = mGroupTwoFinish.p1;
    pos["LineP2"] = mGroupTwoFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);


	finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

double CarCrashSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mCollisions; 
    doubleAcc["FLFitnessWeight"] = 5;
    doubleAcc["ColFitnessWeight"] = 1;
    
    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        pos[mGroupOneAgents[k]] = getPositionInfo(mGroupOneAgents[k]);
    intAcc["Positive"] = 1;
    pos["LineP1"] = mGroupOneFinish.p1;
    pos["LineP2"] = mGroupOneFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);

    pos.clear();

    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        pos[mGroupTwoAgents[k]] = getPositionInfo(mGroupTwoAgents[k]);
    intAcc["Positive"] = 0;
    pos["LineP1"] = mGroupTwoFinish.p1;
    pos["LineP2"] = mGroupTwoFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);


    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

Simulation* CarCrashSimulation::getNewCopy(){
    Simulation* sim = new CarCrashSimulation(mRangefinderRadius, mGroupOneAgents.size(), mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed, mTeamSetup);
    sim->initialise();
    
    return sim;
}

bool CarCrashSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mGroupOneFinish.p1 = vector3(-10, 0, 30); mGroupOneFinish.p2 = vector3(10, 0, 30);
    mGroupTwoFinish.p1 = vector3(-10, 0, -30); mGroupTwoFinish.p2 = vector3(10, 0, -30);

    btQuaternion rotGroupOne(0, 0, 0, 1), rotGroupTwo(0, 0, 0, 1);
    rotGroupOne.setEuler(PI + PI/2, 0, 0);
    rotGroupTwo.setEuler(PI/2, 0, 0);

    double shiftAmount = (double)mGroupOneAgents.size() * 1.5;

    vector3 minDimOne(-8, 0, -36), maxDimOne(8, 0, -30);
    vector3 minDimTwo(-8, 0, 30), maxDimTwo(8, 0, 36);

    boost::mt19937 rng1(mSeed);
    boost::mt19937 rngz1(mSeed + mSeed / 2);
    boost::mt19937 rng2(mSeed * 2);
    boost::mt19937 rngz2(mSeed / 2);

    boost::uniform_real<double> distxone(minDimOne.x, maxDimOne.x);
    boost::uniform_real<double> distzone(minDimOne.z, maxDimOne.z);
    boost::uniform_real<double> distxtwo(minDimTwo.x, maxDimTwo.x);
    boost::uniform_real<double> distztwo(minDimTwo.z, maxDimTwo.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxone(rng1, distxone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genzone(rngz1, distzone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxtwo(rng2, distxtwo);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genztwo(rngz2, distztwo);

    for(uint k = 0; k < mGroupOneAgents.size(); k++){
        mWorldEntities[mGroupOneAgents[k]] = new CarAgent(15, 1);
        if(!mWorldEntities[mGroupOneAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupOne, mResourceManager, vector3(genxone(), 0, genzone()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupOneAgents[k]]->getRigidBody());
    }

    for(uint k = 0; k < mGroupTwoAgents.size(); k++){
        mWorldEntities[mGroupTwoAgents[k]] = new CarAgent(15, 1);
        if(!mWorldEntities[mGroupTwoAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupTwo, mResourceManager, vector3(genxtwo(), 0, genztwo()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupTwoAgents[k]]->getRigidBody());
    }

    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("corridor.mesh", vector3(10, 20, 25), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 2, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;

    return true;
}

vector<Line> CarCrashSimulation::getLines(){
    vector<Line> lines;
    lines.push_back(mGroupOneFinish);
    lines.push_back(mGroupTwoFinish);

    return lines;
}

void CarCrashSimulation::tick(){
    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        mWorldEntities[mGroupOneAgents[k]]->tick();

    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        mWorldEntities[mGroupTwoAgents[k]]->tick();
}

void CarCrashSimulation::applyUpdateRules(string _agentName, uint _groupNum){
    map<uint, double> input;
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));

    double frontDist = d1 > d2 ? d2 : d1;

    //rangefinders
    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 0), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -100), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 100), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0, -100), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, 100), AGENT) / 50;
    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[11] = _groupNum == 1 ? mGroupOneFinish.p1.x / 50 : mGroupTwoFinish.p1.x / 50;
    input[12] = _groupNum == 1 ? mGroupOneFinish.p1.z / 50 : mGroupTwoFinish.p1.z / 50;
    input[13] = _groupNum == 1 ? mGroupOneFinish.p2.x / 50 : mGroupTwoFinish.p2.x / 50;
    input[14] = _groupNum == 1 ? mGroupOneFinish.p2.z / 50 : mGroupTwoFinish.p2.z / 50;

    //velocity
    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[15] = agentVel.x;
    input[16] = agentVel.z;

    mWorldEntities[_agentName]->avoided();
    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
    mWorldEntities[_agentName]->update(output);

    Line finishLine = _groupNum == 0 ? mGroupOneFinish : mGroupTwoFinish;
    double crossVal = calcCrossVal(finishLine.p1, finishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    bool checkCollisions = _groupNum == 0? crossVal < 0 : crossVal > 0;

    if(checkCollisions && mCycleCounter > 10){
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
}

ESPParameters CarCrashSimulation::getESPParams(string _nnFormatFile){
	ESPParameters params;
    params.populationSize = 50;
    params.maxGenerations = 9999999;
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

StandardGAParameters CarCrashSimulation::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 99999;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 999999;
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

CMAESParameters CarCrashSimulation::getCMAESParameters(string _nnFormatFile){
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