#include "evacuationsimulation.h"

EvacuationSimulation::EvacuationSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager), mDeathRng(_seed), deathDist(0, 1), genDeath(mDeathRng, deathDist){
    mWorld->setInternalTickCallback(EvacuationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;
    mAngularVelAcc = 0;

    for(uint k = 0; k < 60; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

EvacuationSimulation::~EvacuationSimulation(){
    
}

EvacuationSimulation::EvacuationSimulation(const EvacuationSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager), mDeathRng(other.mSeed), deathDist(0, 1), genDeath(mDeathRng, deathDist){
    mWorld->setInternalTickCallback(EvacuationSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;
    mAngularVelAcc = 0;

    for(uint k = 0; k < 60; k++)
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

void EvacuationSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    mObjectsToRemove.clear();

    for(int k = 0; k < mAgents.size(); k++)
        applyUpdateRules(mAgents[k], 0);

    //remove dead agents from simulation
    for(uint k = 0; k < mObjectsToRemove.size(); ++k){
        if(mWorldEntities.find(mObjectsToRemove[k]) != mWorldEntities.end()){
            delete mWorldEntities[mObjectsToRemove[k]];
            mWorldEntities.erase(mObjectsToRemove[k]);
        }

        //remove agent from list
        int pos = -1;
        for(uint i = 0; i < mAgents.size(); ++i)
            if(mAgents[i] == mObjectsToRemove[k])
                pos = i;
        if(pos > -1)
            mAgents.erase(mAgents.begin() + pos);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/((float)mCyclesPerSecond));
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

    dblAcc["LowerBound"] = 13;
    dblAcc["UpperBound"] = 15;
    dblAcc["Value"] = mAgents.size();
    dblAcc["EVWeight"] = 30;
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

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
    
    dblAcc["LowerBound"] = 13;
    dblAcc["UpperBound"] = 15;
    dblAcc["Value"] = mAgents.size();
    dblAcc["EVWeight"] = 30;
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

Simulation* EvacuationSimulation::getNewCopy(){
    Simulation* sim = new EvacuationSimulation(*this);
    sim->initialise();
    
    return sim;
}

vector<Line> EvacuationSimulation::getLines(){
    return mLines;
}

bool EvacuationSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new ExpectedValueFitness());

    //change
    mExit.p1 = vector3(-10, 0, -50);
    mExit.p2 = vector3(10, 0, -50);

    mLines.push_back(mExit);

    //set the vals
    vector3 a1minDim(-40, 0, -30), a1maxDim(40, 0, 40);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distxa1(a1minDim.x, a1maxDim.x);
    boost::uniform_real<double> distza1(a1minDim.z, a1maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxa1(rng, distxa1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genza1(rngz, distza1);

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new HumanAgent(15, 2, mCyclesPerSecond);
        
        if(!mWorldEntities[mAgents[k]]->initialise("human.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genxa1(), 0, genza1()), 0.01, mSeed))
            return false;
        mWorldEntities[mAgents[k]]->setAnimationInfo("idle", true);
        
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }


    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("humanevacenv.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 5, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void EvacuationSimulation::tick(){
    for(int k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]]->tick();

        if(mCycleCounter < 10)
            continue;

        btTransform trans;
        mWorldEntities[mAgents[k]]->getRigidBody()->getMotionState()->getWorldTransform(trans);

        if(calcCrossVal(mExit.p1, mExit.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0){
            int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	        for (int i=0;i<numManifolds;i++)
	        {
		        btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
                if(contactManifold->getNumContacts() < 1)
                    continue;

		        const btCollisionObject* obA = contactManifold->getBody0();
		        const btCollisionObject* obB = contactManifold->getBody1();
            }
        }
    }
}

void EvacuationSimulation::applyUpdateRules(string _agentName, uint _group){
    //do nothing if agent still busy with non looping animation
    if(!mWorldEntities[_agentName]->getAnimationLoop())
        return;

    if(mWorldEntities[_agentName]->getLastAnimation() == "staggerforward"){
        //if(genDeath() == 0){
            mWorldEntities[_agentName]->setAnimationInfo("Die", false);
            mWorld->removeRigidBody(mWorldEntities[_agentName]->getRigidBody());
            return;
        /*}
        else{
            mWorldEntities[_agentName]->setAnimationInfo("Recover", false);
            return;
        }*/
    }

    if(mWorldEntities[_agentName]->getLastAnimation() == "Die"){  
        mObjectsToRemove.push_back(_agentName);
        return;
    }

    //query ANN
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

    double angVel = mWorldEntities[_agentName]->getAngularVelocity().y;

    if(calcCrossVal(mExit.p1, mExit.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0)
    {
        if(frontDist < 10){
            mWorldEntities[_agentName]->avoidCollisions(d1, d2, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
        }
        else{
            vector3 hitpos;
            const btCollisionObject* frontObject;
            string frontObjectName = "";

            double frontVal = getRayCollisionDistanceNonEnv(_agentName, btVector3(100, 0.1, 0), frontObject, hitpos) > 2 ? 1 : 0;
           
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
         
            input[17] = angVel;

            //for pushing behavior
            //double density = calculateDensity(_agentName, 10);
            //vector3 avgVel = calculateAverageVelocity(_agentName, 10);
            //vector3 pos = getPositionInfo(_agentName); pos.y = 0;
            //vector3 goalmid = mExit.getMidpoint(); goalmid.y = 0;
            //double distance = pos.calcDistance(goalmid) / 50;

            input[18] = (double)mAgents.size() / 60.0;

            vector<double> output = mSolution->evaluateNeuralNetwork(_group, input);
            output.push_back(frontVal);

            if(frontVal == 0 && output[2] > 0.5){
                for(uint k = 0; k < mAgents.size(); k++){
                    if(frontObject == mWorldEntities[mAgents[k]]->getRigidBody()){
                        frontObjectName = mAgents[k];
                        break;
                    }
                }

                if(frontObjectName == "")
                    cout << "some shitty bug" << endl;
                else{
                    if(mWorldEntities[frontObjectName]->getAnimationLoop() && mWorldEntities[frontObjectName]->getLastAnimation() != "Die" && mWorldEntities[frontObjectName]->getAnimationName() != "" && mWorldEntities[frontObjectName]->getAnimationName() != "staggerforward"){
                        mWorldEntities[frontObjectName]->setAnimationInfo("staggerforward", false);
                        mWorldEntities[frontObjectName]->setVelocity(vector3(0, 0, 0));
                        mWorldEntities[_agentName]->setAnimationInfo("shove", false);
                    }
                }
            }

            mWorldEntities[_agentName]->update(output);
        }
    }

    if(calcCrossVal(mExit.p1, mExit.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0 && mCycleCounter > 10){
        //mAngularVelAcc += fabs(angVel);

        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < mRangefinderRadius)
                mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
    }
}

vector3 EvacuationSimulation::calculateAverageVelocity(string _agentName, double _radius){
    vector3 averageVel;
    int numAgents = 0;
    vector3 currAgentPos = getPositionInfo(_agentName);
    btVector3 orig(1, 0, 0);
    btMatrix3x3& rot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis();
    btVector3 orient = rot * orig;
    vector3 agentOrientation(orient.getX(), orient.getY(), orient.getZ());
 
    for(uint k = 0; k < mAgents.size(); ++k){
        if(_agentName == mAgents[k])
            continue;

        vector3 agentPos = getPositionInfo(mAgents[k]);
        vector3 dirVec = agentPos - currAgentPos;
        if(agentPos.calcDistance(currAgentPos) <= _radius && dirVec.dotValue(vector3(orient.getX(), orient.getY(), orient.getZ())) > 0){
            vector3 bAgentVel = mWorldEntities[_agentName]->getVelocity();
            
            averageVel.x += bAgentVel.x;
            averageVel.z += bAgentVel.z;

            numAgents++;
        }
    }

    if(numAgents != 0){
        averageVel.x /= numAgents;
        averageVel.z /= numAgents;
    }

    return averageVel;
}

double EvacuationSimulation::calculateDensity(string _agentName, double _radius){
    int numAgents = 0;
    vector3 currAgentPos = getPositionInfo(_agentName);
    btVector3 orig(1, 0, 0);
    btMatrix3x3& rot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis();
    btVector3 orient = rot * orig;
    vector3 agentOrientation(orient.getX(), orient.getY(), orient.getZ());

    for(uint k = 0; k < mAgents.size(); ++k){
        if(_agentName == mAgents[k])
            continue;

        vector3 agentPos = getPositionInfo(mAgents[k]);
        vector3 dirVec = agentPos - currAgentPos;

        if(agentPos.calcDistance(currAgentPos) <= _radius && dirVec.dotValue(vector3(orient.getX(), orient.getY(), orient.getZ())) > 0)
            numAgents++;
    }

    double density = (double)numAgents / (double)mAgents.size();

    return density;
}

ESPParameters EvacuationSimulation::getESPParams(string _nnFormatFile){
	ESPParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.maxCompGenerations = 0;
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
    params.sampleEvaluationsPerChromosome = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;
    params.deltaCodeRadius = 0.05;

    return params;
}

StandardGAParameters EvacuationSimulation::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 1000;
    params.fitnessEpsilonThreshold = 0;
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

CMAESParameters EvacuationSimulation::getCMAESParameters(string _nnFormatFile){
    CMAESParameters params;

    params.maxGenerations = 600;
    params.maxCompGenerations = 0;
    params.evalsPerCompChrom = 5;
    params.nnFormatFilename = _nnFormatFile;
    params.fitnessEpsilonThreshold = 0;
    params.deltaCodeRadius = 0.2;
    params.initStepsize = 0.2;

    return params;
}