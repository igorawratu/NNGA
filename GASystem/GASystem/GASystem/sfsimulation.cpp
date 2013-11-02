#include "sfsimulation.h"

SFSimulation::SFSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(SFSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < _numAgents; k++)
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));

    mFitnessFunctions.push_back(new GoalPointFitness());
    //evf?
    mFitnessFunctions.push_back(new CollisionFitness());
}

SFSimulation::SFSimulation(const SFSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(SFSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderRadius = other.mRangefinderRadius;
    mRangefinderVals = 0;

    for(uint k = 0; k < other.mAgents.size(); k++)
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));

    mFitnessFunctions.push_back(new GoalPointFitness());
    //evf?
    mFitnessFunctions.push_back(new CollisionFitness());
}

SFSimulation::~SFSimulation(){}

void SFSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double SFSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> doubleAcc;
    intAcc["Collisions"] = mRangefinderVals + mCollisions; 
    intAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 1;
    doubleAcc["GoalRadius"] = mGoalRadius;
    pos["GoalPoint"] = mGoalpoint;
    

    for(uint k = 0; k < mAgents.size(); k++)
        if(!reached(mAgents[k]))
            pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    //NB: add crowding ff, can use expected value fitness
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc) : 1000; //change this amount

    return finalFitness;
}

vector3 SFSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
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
    intAcc["Collisions"] = mCollisions; 
    intAcc["ColFitnessWeight"] = 1;
    intAcc["Positive"] = 0;

    doubleAcc["GPWeight"] = 1;
    doubleAcc["GoalRadius"] = mGoalRadius;
    pos["GoalPoint"] = mGoalpoint;
    

    for(uint k = 0; k < mAgents.size(); k++)
        if(!reached(mAgents[k]))
            pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    //NB: add crowding ff, can use expected value fitness
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, intAcc) : 1000; //change this amount

    return finalFitness;
}

void SFSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    map<uint, double> input;

    int inputIndex = 1;

    for(int k = -100; k <= 100; k += 50){
        for(int i = -100; i <= 100; i+= 50){
            input[inputIndex] = getRayCollisionDistance(_agentName, btVector3(k, i, 5));
            inputIndex++;
        }
    }

    //agent position
    input[1 + inputIndex] = trans.getOrigin().getX() / 50;
    input[2 + inputIndex] = trans.getOrigin().getY() / 50;
    input[3 + inputIndex] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[4 + inputIndex] = mGoalpoint.x / 50;
    input[5 + inputIndex] = mGoalpoint.y / 50;
    input[6 + inputIndex] = mGoalpoint.z / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[7 + inputIndex] = agentVel.x;
    input[8 + inputIndex] = agentVel.y;
    input[9 + inputIndex] = agentVel.z;

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName]->update(output);

    if(!reached(_agentName) && getPositionInfo(_agentName).calcDistance(mGoalpoint) < mGoalRadius)
        mReached.push_back(_agentName);

    if(!reached(_agentName)){
        for(uint k = 1; k < inputIndex; k++)
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

double SFSimulation::getRayCollisionDistance(string _agentName, const btVector3& _ray){
    double dist = 100;
    btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    mWorld->rayTest(agentPosition, correctedRay, ray);

    vector3 from(agentPosition.getX(), agentPosition.getY(), agentPosition.getZ());
    if(ray.hasHit())
        dist = calcEucDistance(vector3(agentPosition.getX(), agentPosition.getY(), agentPosition.getZ()), vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));

    return dist;
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