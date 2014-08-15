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

    dblAcc["EVWeight"] = 1;
    dblAcc["LowerBound"] = mAgents.size() * (mNumCycles/mCyclesPerSecond) * 8;
    dblAcc["UpperBound"] = mAgents.size() * (mNumCycles/mCyclesPerSecond) * 10;

    double distance = 0;
    for(uint k = 0; k < mAgents.size(); ++k)
        distance += getPositionInfo(mAgents[k]).calcDistance(mCenterPoint);
    dblAcc["Value"] = distance;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : mAgents.size() * (mNumCycles/mCyclesPerSecond) * 8;
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

    dblAcc["EVWeight"] = 1;
    dblAcc["LowerBound"] = mAgents.size() * (mNumCycles/mCyclesPerSecond) * 8;
    dblAcc["UpperBound"] = mAgents.size() * (mNumCycles/mCyclesPerSecond) * 10;

    double distance = 0;
    for(uint k = 0; k < mAgents.size(); ++k)
        distance += getPositionInfo(mAgents[k]).calcDistance(mCenterPoint);
    dblAcc["Value"] = distance;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : mAgents.size() * (mNumCycles/mCyclesPerSecond) * 8;
    //finalFitness += mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

vector<Line> MouseScatterSimulation::getLines(){
    return mLines;
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
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);


    for(uint k = 0; k < mAgents.size(); k++){
        //set rotation to have an orientation facing away from centerpoint
        vector3 currAgentPos(genx(), 0, genz());

        btVector3 agentOrientation(1, 0, 0);
        btVector3 rotVec(currAgentPos.x - mCenterPoint.x, currAgentPos.y - mCenterPoint.y, currAgentPos.z - mCenterPoint.z);

        btQuaternion rot(0, 0, 0, 1);
        btVector3 cross = agentOrientation.cross(rotVec);

        if(cross.length() > 0){
            rot.setX(cross.getX());
            rot.setY(cross.getY());
            rot.setZ(cross.getZ());
            rot.setW(sqrt(agentOrientation.length() * agentOrientation.length() + rotVec.length() * rotVec.length()) + agentOrientation.dot(rotVec));
        }

        /*btVector3 directionVec = btVector3(currAgentPos.x - mCenterPoint.x, 0, currAgentPos.z - mCenterPoint.z);
        btVector3 originalVec = btVector3(1, 0, 0);
        btVector3 cross = directionVec.cross(originalVec);

        Line l1, l2, l3;
        l1.p1 = l2.p1 = l3.p1 = vector3(0, 0, 0);
        l1.p2 = vector3(cross.getX(), cross.getY(), cross.getZ());
        l2.p2 = vector3(originalVec.getX(), originalVec.getY(), originalVec.getZ());
        l3.p2 = vector3(directionVec.getX(), directionVec.getY(), directionVec.getZ());

        mLines.push_back(l1);
        mLines.push_back(l2);
        mLines.push_back(l3);

        if(cross.length() > 0){
            double dot = directionVec.dot(originalVec);
            double mag = directionVec.length() * originalVec.length();

            double c = dot/mag;
            double crossSign = cross < 0 ? -1 : 1;

            //rot.setEuler(acos(c), 0, 0);
            rot.setRotation(btVector3(0, 1, 0), acos(c) * crossSign);
        }*/

        mWorldEntities[mAgents[k]] = new MouseAgent(10, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("mouse.mesh", vector3(1, 1, 1), -rot, mResourceManager, vector3(currAgentPos.x, currAgentPos.y, currAgentPos.z), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("stadium.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mWorldEntities["sphere"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["sphere"]->initialise("sphere.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, mCenterPoint, 0, mSeed))
        return false;

    mInitialised = true;
    
    return true;
}

void MouseScatterSimulation::applyUpdateRules(string _agentName){
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

    double frontDist = d1 > d2 ? d2 : d1;


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

    //possibly add nearest exit?

    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(d1, d2, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{
        mWorldEntities[_agentName]->avoided();
        vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
        output.push_back(frontVal);

        mWorldEntities[_agentName]->update(output);
    }

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

ESPParameters MouseScatterSimulation::getESPParams(string _nnFormatFile){
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

StandardGAParameters MouseScatterSimulation::getSGAParameters(string _nnFormatFile){
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

CMAESParameters MouseScatterSimulation::getCMAESParameters(string _nnFormatFile){
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