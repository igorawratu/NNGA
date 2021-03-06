#include "warrobotsimulation.h"

WarRobotSimulation::WarRobotSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed, TeamSetup _setup)
: Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, _setup){
    mWorld->setInternalTickCallback(WarRobotSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mVelocityAcc = 0;

    for(uint k = 0; k < 40; ++k)
        mGroupOneAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    for(uint k = 40; k < 80; ++k)
        mGroupTwoAgents.push_back("Agent" + boost::lexical_cast<string>(k));
}

WarRobotSimulation::~WarRobotSimulation(){

}

void WarRobotSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    mObjectsToRemove.clear();

    if(mCycleCounter % mCyclesPerDecision == 0){
        mRaysShot.clear();
        if(mCycleCounter % mCyclesPerDecision == 0){
            if(mTeamSetup == TeamSetup::HET){
                for(uint k = 0; k < mGroupOneAgents.size(); ++k){
                    applyUpdateRules(mGroupOneAgents[k], 0, k);
                }
                for(uint k = 0; k < mGroupTwoAgents.size(); ++k){
                    applyUpdateRules(mGroupTwoAgents[k], 1, k);
                }
            }
            else if(mTeamSetup == TeamSetup::QUARTHET){
                for(uint k = 0; k < mGroupOneAgents.size(); ++k){
                    applyUpdateRules(mGroupOneAgents[k], 0, k / 4);
                }
                for(uint k = 0; k < mGroupTwoAgents.size(); ++k){
                    applyUpdateRules(mGroupTwoAgents[k], 1, k / 4);
                }
            }
            else if(mTeamSetup == TeamSetup::SEMIHET){
                for(uint k = 0; k < mGroupOneAgents.size(); ++k){
                    applyUpdateRules(mGroupOneAgents[k], 0, k / 10);
                }
                for(uint k = 0; k < mGroupTwoAgents.size(); ++k){
                    applyUpdateRules(mGroupTwoAgents[k], 1, k / 10);
                }
            }
            else if(mTeamSetup == TeamSetup::HOM){
                for(uint k = 0; k < mGroupOneAgents.size(); ++k){
                    applyUpdateRules(mGroupOneAgents[k], 0, 0);
                }
                for(uint k = 0; k < mGroupTwoAgents.size(); ++k){
                    applyUpdateRules(mGroupTwoAgents[k], 1, 0);
                }
            }
        }

        //remove dead agents from simulation
        for(uint k = 0; k < mObjectsToRemove.size(); ++k){
            if(mWorldEntities.find(mObjectsToRemove[k]) != mWorldEntities.end()){
                mWorld->removeRigidBody(mWorldEntities[mObjectsToRemove[k]]->getRigidBody());
                delete mWorldEntities[mObjectsToRemove[k]];
                mWorldEntities.erase(mObjectsToRemove[k]);
            }

            //remove agent from lists
            int pos = -1;
            for(uint i = 0; i < mGroupOneAgents.size(); ++i)
                if(mGroupOneAgents[i] == mObjectsToRemove[k])
                    pos = i;
            if(pos > -1)
                mGroupOneAgents.erase(mGroupOneAgents.begin() + pos);
            else{
                for(uint i = 0; i < mGroupTwoAgents.size(); ++i)
                    if(mGroupTwoAgents[i] == mObjectsToRemove[k])
                        pos = i;
                if(pos > -1)
                    mGroupTwoAgents.erase(mGroupTwoAgents.begin() + pos);
            }
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double WarRobotSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;

    dblAcc["LowerBound"] = 9;
    dblAcc["UpperBound"] = 11;
    dblAcc["Value"] = mGroupOneAgents.size();
    dblAcc["EVWeight"] = 20;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["LowerBound"] = 9;
    dblAcc["UpperBound"] = 11;
    dblAcc["Value"] = mGroupTwoAgents.size();
    dblAcc["EVWeight"] = 20;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["Collisions"] = mRangefinderVals + mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

Simulation* WarRobotSimulation::getNewCopy(){
    Simulation* sim = new WarRobotSimulation(mRangefinderRadius, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed, mTeamSetup);
    sim->initialise();
    
    return sim;
}

bool WarRobotSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new ExpectedValueFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    btQuaternion rotG1(0, 0, 0, 1), rotG2(0, 0, 0, 1);
    rotG1.setEuler(0, 0, 0); rotG2.setEuler(PI, 0, 0);

    vector3 minDimOne(-80, 0, -80), maxDimOne(-30, 0, 80);
    vector3 minDimTwo(30, 0, -80), maxDimTwo(80, 0, 80);

    boost::mt19937 rngonex(mSeed);
    boost::mt19937 rngonez(mSeed + mSeed / 2);
    boost::mt19937 rngtwox(mSeed / 2);
    boost::mt19937 rngtwoz(mSeed * 2);

    boost::uniform_real<double> distxone(minDimOne.x, maxDimOne.x);
    boost::uniform_real<double> distzone(minDimOne.z, maxDimOne.z);
    boost::uniform_real<double> distxtwo(minDimTwo.x, maxDimTwo.x);
    boost::uniform_real<double> distztwo(minDimTwo.z, maxDimTwo.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxone(rngonex, distxone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genzone(rngonez, distzone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxtwo(rngtwox, distxtwo);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genztwo(rngtwoz, distztwo);

    for(uint k = 0; k < mGroupOneAgents.size(); ++k){
        mWorldEntities[mGroupOneAgents[k]] = new WarRobotAgent(15, vector3(10, 0, 10), vector3(-10, 0, -10), 1, 15);
        if(!mWorldEntities[mGroupOneAgents[k]]->initialise("warrobotr.mesh", vector3(1, 1, 1), rotG1, mResourceManager, vector3(genxone(), 0, genzone()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupOneAgents[k]]->getRigidBody());
    }
    
    for(uint k = 0; k < mGroupTwoAgents.size(); ++k){
        mWorldEntities[mGroupTwoAgents[k]] = new WarRobotAgent(15, vector3(10, 0, 10), vector3(-10, 0, -10), 1, 15);
        if(!mWorldEntities[mGroupTwoAgents[k]]->initialise("warrobotb.mesh", vector3(1, 1, 1), rotG2, mResourceManager, vector3(genxtwo(), 0, genztwo()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupTwoAgents[k]]->getRigidBody());
    }
    
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("city.mesh", vector3(100, 100, 100), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;

    return true;
}

void WarRobotSimulation::tick(){
    for(uint k = 0; k < mGroupOneAgents.size(); ++k)
        mWorldEntities[mGroupOneAgents[k]]->tick();

    for(uint k = 0; k < mGroupTwoAgents.size(); ++k)
        mWorldEntities[mGroupTwoAgents[k]]->tick();
}

double WarRobotSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;

    dblAcc["LowerBound"] = 9;
    dblAcc["UpperBound"] = 11;
    dblAcc["Value"] = mGroupOneAgents.size();
    dblAcc["EVWeight"] = 20;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["LowerBound"] = 9;
    dblAcc["UpperBound"] = 11;
    dblAcc["Value"] = mGroupTwoAgents.size();
    dblAcc["EVWeight"] = 20;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["Collisions"] = mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    finalFitness += finalFitness == mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

void WarRobotSimulation::checkRayObject(int _groupNum, const btCollisionObject* _obj, int& _team, string& _entityName){
    for(uint k = 0; k < mGroupOneAgents.size(); k++){
        if(_obj == mWorldEntities[mGroupOneAgents[k]]->getRigidBody()){
            _team = _groupNum == 0 ? -1 : 1;
            _entityName = mGroupOneAgents[k];
            return;
        }
    }

    for(uint k = 0; k < mGroupTwoAgents.size(); k++){
        if(_obj == mWorldEntities[mGroupTwoAgents[k]]->getRigidBody()){
            _team = _groupNum == 1 ? -1 : 1;
            _entityName = mGroupTwoAgents[k];
            return;
        }
    }

    _team = 0;
    _entityName = "env";
}

void WarRobotSimulation::applyUpdateRules(string _agentName, uint _groupNum, uint _index){
    cout << _groupNum << " " << _index;
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    const btCollisionObject* obj;
    const btCollisionObject* front;
    string colliderName, otherColliderName;
    int teamInd, frontTeamInd;
    vector3 hitposfront, hitposother;

    map<uint, double> input;

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = Simulation::getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = Simulation::getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));

    double frontDist = d1 > d2 ? d2 : d1;


    //rangefinders
    /*if(_groupNum < 4)
        input[1] = getRayCollisionDistanceNonEnv(_agentName, btVector3(100, 0, 0), front, hitposfront) / 50;*/
    /*else */input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), front, hitposfront) / 50;
    checkRayObject(_groupNum, front, frontTeamInd, colliderName);
    input[13] = frontTeamInd;

    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 0), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[14] = teamInd;

    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[15] = teamInd;

    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[16] = teamInd;

    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[17] = teamInd;

    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[18] = teamInd;

    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0, -100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[19] = teamInd;

    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, 100), obj, hitposother) / 50;
    checkRayObject(_groupNum, obj, teamInd, otherColliderName);
    input[20] = teamInd;

    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;

    //velocity
    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[11] = agentVel.x;
    input[12] = agentVel.z;

    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(d2, d1, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{
        mWorldEntities[_agentName]->avoided();
        uint team = _groupNum == 0 ? 1 : 2;
        vector<double> output = mSolution->evaluateNeuralNetwork(_index, input, team);
        mWorldEntities[_agentName]->update(output);
    }

    //shooting logic here
    if(frontTeamInd == 1){
        Line ray;
        //first ray point
        ray.p1 = getPositionInfo(_agentName);
        ray.p2 = hitposfront;

        if(ray.p1.calcDistance(ray.p2) < 40 && dynamic_cast<WarRobotAgent*>(mWorldEntities[_agentName])->shootRay()){
            mRaysShot.push_back(ray);
            mObjectsToRemove.push_back(colliderName);
        }
    }

    //fitness eval code
    //try make aggresors move more/faster
    /*if(_groupNum > 3)
        mVelocityAcc += mWorldEntities[_agentName]->getVelocity().calcDistance(vector3(0, 0, 0));*/

    //rangefinder vals
    if(mCycleCounter > 10){
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

vector<CompetitiveFitness> WarRobotSimulation::competitiveFitness(){
    vector<CompetitiveFitness> fitnesses;

    double team1Fitness = 40 - mGroupOneAgents.size() + mGroupTwoAgents.size();
    double team2Fitness = 40 - mGroupTwoAgents.size() + mGroupOneAgents.size();

    fitnesses.push_back(make_pair(1, team1Fitness));
    fitnesses.push_back(make_pair(2, team2Fitness));

    return fitnesses;
}

ESPParameters WarRobotSimulation::getESPParams(string _nnFormatFile){
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
    params.deltaCodeRadius = 0.2;

    return params;
}

StandardGAParameters WarRobotSimulation::getSGAParameters(string _nnFormatFile){
	StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 99999;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = 10000;
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

CMAESParameters WarRobotSimulation::getCMAESParameters(string _nnFormatFile){
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