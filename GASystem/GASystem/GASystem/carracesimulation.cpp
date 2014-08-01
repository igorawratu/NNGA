#include "carracesimulation.h"

CarRaceSimulation::CarRaceSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
: Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CarRaceSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < 10; ++k)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

CarRaceSimulation::~CarRaceSimulation(){
    
}

void CarRaceSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(uint k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k], k);
        //applyUpdateRules(mAgents[0], 0);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CarRaceSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mRangefinderVals + mCollisions; 
    doubleAcc["FLFitnessWeight"] = 1;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["WinnerFitnessWeight"] = 1;
    
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);
    intAcc["Positive"] = 0;
    intAcc["Winner"] = mWinner;
    intAcc["ExpectedWinner"] = 0;
    doubleAcc["WinnerVal"] = mWinToExpectedDistance;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    //finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc) : 1000;

    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);
    //finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

Simulation* CarRaceSimulation::getNewCopy(){
    Simulation* sim = new CarRaceSimulation(mRangefinderRadius, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    sim->initialise();
    
    return sim;
}

bool CarRaceSimulation::initialise(){
    if(mInitialised)
        return true;

    mWinToExpectedDistance = 200;
    
    mWinner = -1;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new WinnerFitness());
    mFitnessFunctions.push_back(new CollisionFitness());
    

    //set finish like here
    mFinishLine.p1.x = -20;
    mFinishLine.p1.y = 0;
    mFinishLine.p1.z = -40;

    mFinishLine.p2.x = 20;
    mFinishLine.p2.y = 0;
    mFinishLine.p2.z = -40;

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    mWorldEntities[mAgents[0]] = new CarAgent(10, 0.5);
    if(!mWorldEntities[mAgents[0]]->initialise("carone.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(-7.5, 0, 38), 0.01, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities[mAgents[0]]->getRigidBody());
    //mWorldEntities[mAgents[0]]->setVelocity(vector3(5, 0, 0));

    for(uint k = 1; k < mAgents.size(); k++){
        vector3 pos;
        if(k < 6){
            pos.x = ((double)k - 1) * 3 - 7.5;
            pos.y = 0;
            pos.z = 34;
        }
        else{
            pos.x = ((double)k - 5) * 3 - 7.5;
            pos.y = 0;
            pos.z = 38;
        }

        mWorldEntities[mAgents[k]] = new CarAgent(10, 0.5);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01, mSeed))
            return false;
        mWorldEntities[mAgents[k]]->setVelocity(vector3(0, 0, 0.5));
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
        //mWorldEntities[mAgents[k]]->setVelocity(vector3(5, 0, 0));
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("racetrack.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 3, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;

    return true;
}

void CarRaceSimulation::tick(){
    for(uint k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}

double CarRaceSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mCollisions; 
    doubleAcc["FLFitnessWeight"] = 1;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["WinnerFitnessWeight"] = 1;
    
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);
    intAcc["Positive"] = 0;
    intAcc["Winner"] = mWinner;
    intAcc["ExpectedWinner"] = 0;
    doubleAcc["WinnerVal"] = mWinToExpectedDistance;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    //finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc) : 1000;

    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);
    //finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

void CarRaceSimulation::applyUpdateRules(string _agentName, uint _groupNum){
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

    map<uint, double> input;
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
    input[11] = mFinishLine.p1.x / 50;
    input[12] = mFinishLine.p1.z / 50;
    input[13] = mFinishLine.p2.x / 50;
    input[14] = mFinishLine.p2.z / 50;

    //velocity
    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[15] = agentVel.x;
    input[16] = agentVel.z;

    input[17] = mWorldEntities[mAgents[0]]->getVelocity().x;
    input[18] = mWorldEntities[mAgents[0]]->getVelocity().z;

    btTransform winnerTrans;
    mWorldEntities[mAgents[0]]->getRigidBody()->getMotionState()->getWorldTransform(winnerTrans);
    input[19] = winnerTrans.getOrigin().getX() / 50;
    input[20] = winnerTrans.getOrigin().getZ() / 50;
    input[21] = _groupNum == 0 ? -1 : 1;

    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(d2, d1, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{
        mWorldEntities[_agentName]->avoided();
        //vector<double> output = mSolution->evaluateNeuralNetwork(_groupNum, input);
        vector<double> output = mSolution->evaluateNeuralNetwork(0, input, _groupNum + 1);
        mWorldEntities[_agentName]->update(output);
    }

    bool checkCollisions = calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0;

    if(!checkCollisions && mWinner == -1){
        for(uint k = 0; k < mAgents.size(); k++){
            if(mAgents[k] == _agentName){
                btTransform winnerTrans;
                mWorldEntities[mAgents[0]]->getRigidBody()->getMotionState()->getWorldTransform(winnerTrans);
           
                mWinner = k;
                mWinToExpectedDistance = vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()).calcDistance(vector3(winnerTrans.getOrigin().getX(), winnerTrans.getOrigin().getY(), winnerTrans.getOrigin().getZ()));
                break;
            }
        }

        for(uint k = 0; k < mAgents.size(); ++k){
            mAgentDistances.push_back(getPositionInfo(mAgents[k]).calcDistance(getPositionInfo(mAgents[mWinner])));
        }
    }

    if(checkCollisions && mCycleCounter > 10){
        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < mRangefinderRadius)
                mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
        
        //gets collision data
        int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	    for(int i=0;i<numManifolds;i++)
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

vector<CompetitiveFitness> CarRaceSimulation::competitiveFitness(){
    vector<CompetitiveFitness> fitnesses;
    if(mAgentDistances.size() == 0){
        vector3 midpoint((mFinishLine.p1.x + mFinishLine.p2.x)/2, (mFinishLine.p1.y + mFinishLine.p2.y)/2, (mFinishLine.p1.z + mFinishLine.p2.z)/2);

        int closestPos;
        double closestDist = 9999999;

        for(uint k = 0; k < mAgents.size(); ++k){
            double currDist = midpoint.calcDistance(getPositionInfo(mAgents[k]));

            if(closestDist > currDist){
                closestDist = currDist;
                closestPos = k;
            }
        }

        for(uint k = 0; k < mAgents.size(); ++k){
            double currAgentFit = getPositionInfo(mAgents[k]).calcDistance(getPositionInfo(mAgents[closestPos]));
            fitnesses.push_back(make_pair(k + 1, currAgentFit));
        }
    }
    else{
        for(uint k = 0; k < mAgentDistances.size(); ++k){
            fitnesses.push_back(make_pair(k + 1, mAgentDistances[k]));
        }
    }

    return fitnesses;
}

ESPParameters CarRaceSimulation::getESPParams(string _nnFormatFile){
	ESPParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.maxCompGenerations = 400;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.1;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "LX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = params.populationSize/10;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;
    params.deltaCodeRadius = 0.1;

    return params;
}

StandardGAParameters CarRaceSimulation::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 50;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.1;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "LX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = params.populationSize/10;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    return params;
}