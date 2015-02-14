#include "sfsimulation.h"

SFSimulation::SFSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(SFSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    mFitnessFunctions.push_back(new GoalPointFitness());
    mFitnessFunctions.push_back(new CollisionFitness());
    mFitnessFunctions.push_back(new ExpectedValueFitness());
    mAngularVelAcc = 0;
}

SFSimulation::SFSimulation(const SFSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(SFSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    mFitnessFunctions.push_back(new GoalPointFitness());
    mFitnessFunctions.push_back(new CollisionFitness());
    mFitnessFunctions.push_back(new ExpectedValueFitness());
    mAngularVelAcc = 0;
}

SFSimulation::~SFSimulation(){}

void SFSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++){
            int group = k / 10;
            applyUpdateRules(mAgents[k], 0);
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/((float)mCyclesPerSecond));
}

vector3 SFSimulation::calculateCentroid(){
    vector3 result(0, 0, 0);

    for(int k = 0; k < mAgents.size(); ++k){
        vector3 agentPos = getPositionInfo(mAgents[k]);
        result.x += agentPos.x;
        result.y += agentPos.y;
        result.z += agentPos.z;
    }

    result.x /= mAgents.size();
    result.y /= mAgents.size();
    result.z /= mAgents.size();

    return result;
}

double SFSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mRangefinderVals + mCollisions; 
    doubleAcc["ColFitnessWeight"] = 10;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 2;
    doubleAcc["GoalRadius"] = mGoalRadius;
    pos["GoalPoint"] = mGoalpoint;
    pos["Agent0"] = calculateCentroid();
    
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);

    doubleAcc["GoalRadius"] = mCrowdingRadius;

    for(uint k = 0; k < mAgents.size(); k++)
        if(!reached(mAgents[k]))
            pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);

    doubleAcc["LowerBound"] = 0;
    doubleAcc["UpperBound"] = (mAgents.size() * mNumCycles / mCyclesPerDecision) / 4;
    doubleAcc["Value"] = mAngularVelAcc;
    doubleAcc["EVWeight"] = 0.5;

    finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc);

    return finalFitness;
}

void SFSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}

double SFSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    doubleAcc["Collisions"] = mCollisions; 
    doubleAcc["ColFitnessWeight"] = 10;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 2;
    doubleAcc["GoalRadius"] = mGoalRadius;
    pos["GoalPoint"] = mGoalpoint;
    pos["Agent0"] = calculateCentroid();
    
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);

    doubleAcc["GoalRadius"] = mCrowdingRadius;

    for(uint k = 0; k < mAgents.size(); k++)
        if(!reached(mAgents[k]))
            pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc);

    doubleAcc["LowerBound"] = 0;
    doubleAcc["UpperBound"] = (mAgents.size() * mNumCycles / mCyclesPerDecision) / 4;
    doubleAcc["Value"] = mAngularVelAcc;
    doubleAcc["EVWeight"] = 0.5;

    finalFitness += mFitnessFunctions[2]->evaluateFitness(pos, doubleAcc, intAcc);

    return finalFitness;
}

void SFSimulation::applyUpdateRules(string _agentName, int _groupNum){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    map<uint, double> input;

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));
    double d3 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, agentBox->getHalfExtentsWithMargin().getZ(), 0));
    double d4 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, -agentBox->getHalfExtentsWithMargin().getZ(), 0));

    double semifinDist1 = d1 > d2 ? d2 : d1;
    double semifinDist2 = d3 > d4 ? d4 : d3;
    double frontDist = semifinDist1 > semifinDist2 ? semifinDist2 : semifinDist1;

    int inputIndex = 1;

    for(int k = -100; k <= 100; k += 50){
        for(int i = -100; i <= 100; i+= 50){
            input[inputIndex++] = getRayCollisionDistance(_agentName, btVector3(k, i, 5), AGENT) / 50;
        }
    }

    //agent position
    input[inputIndex] = trans.getOrigin().getX() / 50;
    input[1 + inputIndex] = trans.getOrigin().getY() / 50;
    input[2 + inputIndex] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[3 + inputIndex] = mGoalpoint.x / 50;
    input[4 + inputIndex] = mGoalpoint.y / 50;
    input[5 + inputIndex] = mGoalpoint.z / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[6 + inputIndex] = agentVel.x;
    input[7 + inputIndex] = agentVel.y;
    input[8 + inputIndex] = agentVel.z;

    vector3 angVel = mWorldEntities[_agentName]->getAngularVelocity();
    double angVelDist = angVel.calcDistance(vector3(0, 0, 0));
    input[9 + inputIndex] = angVelDist;

    /*if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(frontDist, 0, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{*/
        vector<double> output = mSolution->evaluateNeuralNetwork(_groupNum, input);

        mWorldEntities[_agentName]->update(output);
    //}

    if(!reached(_agentName) && getPositionInfo(_agentName).calcDistance(mGoalpoint) < mCrowdingRadius)
        mReached.push_back(_agentName);

    if(!reached(_agentName) && mCycleCounter > 10){
        mAngularVelAcc += angVelDist;

        for(uint k = 1; k < inputIndex; k++){
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
            
            if((mWorldEntities[_agentName]->getRigidBody() == obA || mWorldEntities[_agentName]->getRigidBody() == obB))
                mCollisions++;
        }
    }
}

bool SFSimulation::reached(string _agentName){
    bool found = false;
    for(uint k = 0; k < mReached.size(); ++k){
        if(mReached[k] == _agentName){
            found = true;
            break;
        }
    }

    return found;
}

ESPParameters SFSimulation::getESPParams(string _nnFormatFile){
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

StandardGAParameters SFSimulation::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 9999999;
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

CMAESParameters SFSimulation::getCMAESParameters(string _nnFormatFile){
    CMAESParameters params;

    params.maxGenerations = 9999999;
    params.maxCompGenerations = 0;
    params.evalsPerCompChrom = 5;
    params.nnFormatFilename = _nnFormatFile;
    params.fitnessEpsilonThreshold = -1;
    params.deltaCodeRadius = 0.2;
    params.initStepsize = 0.2;

    return params;
}