#include "bridgesimulation.h"

BridgeSimulation::BridgeSimulation(double _rangefinderRadius, uint _numAgents, Line _finishLine, AgentType _agentType, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(BridgeSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mAgentType = _agentType;
    mFinishLine = _finishLine;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));
}

BridgeSimulation::~BridgeSimulation(){}

void BridgeSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
#ifdef MT
        #pragma omp parallel for
#endif
        for(int k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double BridgeSimulation::fitness(vector<Fitness*> _fit){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    intAcc["Collisions"] = (mRangefinderVals + mCollisions) / 5; 
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    for(uint k = 0; k < _fit.size(); k++)
        finalFitness += _fit[k]->evaluateFitness(pos, map<string, double>(), intAcc);

    return finalFitness;
}

vector3 BridgeSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}

Simulation* BridgeSimulation::getNewCopy(){
    Simulation* newsim = new BridgeSimulation(mRangefinderRadius, mAgents.size(), mFinishLine, mAgentType, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    newsim->initialise();

    return newsim;
}

void BridgeSimulation::tick(){
#ifdef MT
    #pragma omp parallel for
#endif
    for(int k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}

bool BridgeSimulation::initialise(){
    if(mInitialised)
        return true;

    //set the vals
    vector3 minDim(-22, -47, 20), maxDim(22, -46.9, 35);

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
            if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), geny(), genz()), 0.01))
                return false;
            mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
        }
    }
    else if(mAgentType == MOUSE){
        for(uint k = 0; k < mAgents.size(); k++){
            mWorldEntities[mAgents[k]] = new MouseAgent(3, 10, 0.5);
            if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), geny(), genz()), 0.01))
                return false;
            mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
        }
    }
    else{
        cerr << "Error: unrecognised agent type" << endl;
        return false;
    }
    
    mWorldEntities["bridge"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["bridge"]->initialise("bridge.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["bridge"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void BridgeSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    map<uint, double> input;
    //rangefinders
    if(mAgentType == CAR){
        input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 0)) / 50;
        input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 0)) / 50;
        input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100)) / 50;
        input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100)) / 50;
        input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -100)) / 50;
        input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 100)) / 50;
        input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0, -100)) / 50;
        input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, 100)) / 50;
    }
    else if (mAgentType == MOUSE){
        input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 105)) / 50;
        input[2] = getRayCollisionDistance(_agentName, btVector3(100, 0, 75)) / 50;
        input[3] = getRayCollisionDistance(_agentName, btVector3(100, 0, 45)) / 50;
        input[4] = getRayCollisionDistance(_agentName, btVector3(100, 0, 15)) / 50;
        input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -15)) / 50;
        input[6] = getRayCollisionDistance(_agentName, btVector3(100, 0, -45)) / 50;
        input[7] = getRayCollisionDistance(_agentName, btVector3(100, 0, -75)) / 50;
        input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, -105)) / 50;
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

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName]->update(output);

    if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0){
        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < 5)
                mRangefinderVals += 1 - (input[k] * 50)/5;
        
        //gets collision data
        int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	    for (int i=0;i<numManifolds;i++)
	    {
		    btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
		    const btCollisionObject* obA = contactManifold->getBody0();
		    const btCollisionObject* obB = contactManifold->getBody1();

            if(mWorldEntities[_agentName]->getRigidBody() == obA || mWorldEntities[_agentName]->getRigidBody() == obB)
                mCollisions++;
        }
    }
}

double BridgeSimulation::getRayCollisionDistance(string _agentName, const btVector3& _ray){
    double dist = 100;
    btVector3 correctedRay = _ray * mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis();

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    vector3 from(agentPosition.getX(), agentPosition.getY(), agentPosition.getZ());
    if(ray.hasHit())
        dist = calcEucDistance(vector3(agentPosition.getX(), agentPosition.getY(), agentPosition.getZ()), vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));

    return dist;
}

double BridgeSimulation::calcDistance(vector3 _from, vector3 _to){
    double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

    return sqrt(x*x + y*y + z*z);
}