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

EvacuationSimulation::EvacuationSimulation(const EvacuationSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
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

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/((float)mCyclesPerSecond));
}

double EvacuationSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["FLFitnessWeight"] = 1;
    intAcc["Positive"] = 0;
    dblAcc["ColFitnessWeight"] = 1;
    dblAcc["Collisions"] = mCollisions + mRangefinderVals; 
    pos["LineP1"] = mExit.p1;
    pos["LineP2"] = mExit.p2;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

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
    dblAcc["ColFitnessWeight"] = 1;
    dblAcc["Collisions"] = mCollisions; 
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc);

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
    mFitnessFunctions.push_back(new CollisionFitness());

    //change
    mExit.p1 = vector3(-10, 0, -50);
    mExit.p2 = vector3(10, 0, -50);

    mLines.push_back(mExit);

    //set the vals
    vector3 a1minDim(-40, 0, -30), a1maxDim(40, 0, 40);
    /*vector3 a1minDim(-40, 0, -40), a1maxDim(-20, 0, 40);
    vector3 a2minDim(20, 0, -40), a2maxDim(40, 0, 40);
    vector3 a3minDim(-20, 0, 20), a3maxDim(20, 0, 40);
    vector3 a4minDim(-20, 0, -40), a4maxDim(20, 0, -20);*/

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);
    //boost::mt19937 rnga(mSeed*2);

    boost::uniform_real<double> distxa1(a1minDim.x, a1maxDim.x);
    boost::uniform_real<double> distza1(a1minDim.z, a1maxDim.z);
    /*boost::uniform_real<double> distxa2(a2minDim.x, a2maxDim.x);
    boost::uniform_real<double> distza2(a2minDim.z, a2maxDim.z);
    boost::uniform_real<double> distxa3(a3minDim.x, a3maxDim.x);
    boost::uniform_real<double> distza3(a3minDim.z, a3maxDim.z);
    boost::uniform_real<double> distxa4(a4minDim.x, a4maxDim.x);
    boost::uniform_real<double> distza4(a4minDim.z, a4maxDim.z);
    boost::uniform_int<> area(0, 3);*/

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxa1(rng, distxa1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genza1(rngz, distza1);
    /*boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxa2(rng, distxa2);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genza2(rngz, distza2);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxa3(rng, distxa3);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genza3(rngz, distza3);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxa4(rng, distxa4);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genza4(rngz, distza4);
    boost::variate_generator<boost::mt19937, boost::uniform_int<>> genarea(rnga, area);*/

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new HumanAgent(10, 2);
        /*int area = genarea();
        vector3 pos;
        switch(area){
            case 0: pos = vector3(genxa1(), 0, genza1());
                break;
            case 1: pos = vector3(genxa2(), 0, genza2());
                break;
            case 2: pos = vector3(genxa3(), 0, genza3());
                break;
            case 3: pos = vector3(genxa4(), 0, genza4());
                break;
            default: pos = vector3(0, 0, 0);
                break;
        }*/
        
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
                
                /*Agent* currAgent = mWorldEntities[mAgents[k]];

                if((currAgent->getRigidBody() == obA || currAgent->getRigidBody() == obB)){
                    mCollisions++;
                    if(!currAgent->getAnimationLoop())
                        continue;

                    if((mWorldEntities["environment"]->getRigidBody() == obA || mWorldEntities["environment"]->getRigidBody() == obB))
                        currAgent->setAnimationInfo("staggerback", false);
                    else{
                        const btCollisionObject* other = currAgent->getRigidBody() == obA ? obB : obA;
                        bool found = false;
                        for(int j = 0; j < mAgents.size(); ++j){
                            if(mWorldEntities[mAgents[j]]->getRigidBody() == other){
                                //get orientation of agent for dot product
                                vector3 orientation = currAgent->getVelocity();
                                vector3 otherorientation = mWorldEntities[mAgents[j]]->getVelocity();
                                vector3 position = getPositionInfo(mAgents[k]);
                                vector3 otherPosition = getPositionInfo(mAgents[j]);

                                //check if agent is stationary
                                if(orientation.calcDistance(vector3(0, 0, 0)) == 0){
                                    if(mWorldEntities[mAgents[j]]->getAnimationLoop())
                                        mWorldEntities[mAgents[j]]->setAnimationInfo("shove", false);
                                    if(orientation.dotValue(otherorientation) < 0)
                                        currAgent->setAnimationInfo("staggerback", false);
                                    else currAgent->setAnimationInfo("staggerforward", false);
                                }
                                else{
                                    vector3 resultant = otherPosition - position;
                                    double dot = resultant.dotValue(orientation);

                                    if(dot > 0){
                                        if(mWorldEntities[mAgents[j]]->getAnimationLoop()){
                                            if(orientation.dotValue(otherorientation) < 0)
                                                mWorldEntities[mAgents[j]]->setAnimationInfo("staggerback", false);
                                            else mWorldEntities[mAgents[j]]->setAnimationInfo("staggerforward", false);
                                        }
                                        currAgent->setAnimationInfo("shove", false);
                                    }
                                    else{
                                        if(mWorldEntities[mAgents[j]]->getAnimationLoop())
                                            mWorldEntities[mAgents[j]]->setAnimationInfo("shove", false);
                                        currAgent->setAnimationInfo("staggerforward", false);
                                    }
                                }
                                found = true;
                                break;
                            }
                        }
                        if(!found){
                            cout << "Error: shit collided with some unknown object" << endl;
                            currAgent->setAnimationInfo("staggerback", false);
                        }
                    }
                }*/
            }
        }
    }
}

void EvacuationSimulation::applyUpdateRules(string _agentName, uint _group){
    //do nothing if agent still busy with non looping animation
    if(!mWorldEntities[_agentName]->getAnimationLoop())
        return;

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

            double frontVal = getRayCollisionDistanceNonEnv(_agentName, btVector3(100, 0.1, 0), frontObject, hitpos) > 1 ? 1 : 0;
           
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
            double density = calculateDensity(_agentName, 10);
            vector3 avgVel = calculateAverageVelocity(_agentName, 10);
            vector3 pos = getPositionInfo(_agentName); pos.y = 0;
            vector3 goalmid = mExit.getMidpoint(); goalmid.y = 0;
            double distance = pos.calcDistance(goalmid) / 50;

            input[18] = distance;
            input[19] = avgVel.x;
            input[20] = avgVel.z;
            input[21] = density;

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
                    btVector3 uncorrectedForce(2, 0, 0);
                    btVector3 relPos = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getOrigin();
                    btMatrix3x3& rot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis();
                    btVector3 force = rot * uncorrectedForce;
                    force.setY(0);

                    mWorldEntities[frontObjectName]->getRigidBody()->applyForce(force, relPos);
                }

                mWorldEntities[_agentName]->setAnimationInfo("shove", false);
            }

            mWorldEntities[_agentName]->update(output);
        }
    }

    if(calcCrossVal(mExit.p1, mExit.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0 && mCycleCounter > 10){
        mAngularVelAcc += fabs(angVel);

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
    params.populationSize = 20;
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