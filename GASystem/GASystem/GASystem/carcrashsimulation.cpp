#include "carcrashsimulation.h"

CarCrashSimulation::CarCrashSimulation(double _rangefinderRadius, uint _agentsPerSide, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
 : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CarCrashSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < _agentsPerSide; k++)
        mGroupOneAgents.push_back("agent" + boost::lexical_cast<string>(k));

    for(uint k = _agentsPerSide; k < _agentsPerSide * 2; k++)
        mGroupTwoAgents.push_back("agent" + boost::lexical_cast<string>(k));
}

CarCrashSimulation::~CarCrashSimulation(){

}

void CarCrashSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(uint k = 0; k < mGroupOneAgents.size(); k++)
            applyUpdateRules(mGroupOneAgents[k], 1);
        for(uint k = 0; k < mGroupTwoAgents.size(); k++)
            applyUpdateRules(mGroupTwoAgents[k], 2);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CarCrashSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    intAcc["Collisions"] = mRangefinderVals + mCollisions; 
    intAcc["FLFitnessWeight"] = 1;
    intAcc["ColFitnessWeight"] = 1;
    
    
    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        pos[mGroupOneAgents[0]] = getPositionInfo(mGroupOneAgents[k]);
    intAcc["Positive"] = 0;
    pos["LineP1"] = mGroupOneFinish.p1;
    pos["LineP2"] = mGroupOneFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, map<string, double>(), intAcc);

    pos.clear();

    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        pos[mGroupTwoAgents[k]] = getPositionInfo(mGroupTwoAgents[k]);
    intAcc["Positive"] = 1;
    pos["LineP1"] = mGroupTwoFinish.p1;
    pos["LineP2"] = mGroupTwoFinish.p2;
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, map<string, double>(), intAcc);


    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc) : 1000;

    return finalFitness;
}

double CarCrashSimulation::realFitness(){
    return fitness() - mRangefinderVals;
}

Simulation* CarCrashSimulation::getNewCopy(){
    Simulation* sim = new CarCrashSimulation(mRangefinderRadius, mGroupOneAgents.size(), mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    sim->initialise();
    
    return sim;
}

bool CarCrashSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mGroupOneFinish.p1 = vector3(-20, 0, 30); mGroupOneFinish.p2 = vector3(20, 0, 30);
    mGroupTwoFinish.p1 = vector3(-20, 0, -30); mGroupTwoFinish.p2 = vector3(20, 0, -30);

    btQuaternion rotGroupOne(0, 0, 0, 1), rotGroupTwo(0, 0, 0, 1);
    rotGroupOne.setEuler(PI/2, 0, 0);
    rotGroupTwo.setEuler(PI + PI/2, 0, 0);

    for(uint k = 0; k < mGroupOneAgents.size(); k++){
        mWorldEntities[mGroupOneAgents[k]] = new CarAgent(10, 1);
        if(!mWorldEntities[mGroupOneAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupOne, mResourceManager, vector3(k, 0, -30), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupOneAgents[k]]->getRigidBody());
    }

    for(uint k = 0; k < mGroupTwoAgents.size(); k++){
        mWorldEntities[mGroupTwoAgents[k]] = new CarAgent(10, 1);
        if(!mWorldEntities[mGroupTwoAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupTwo, mResourceManager, vector3(k, 0, 30), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupTwoAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["corridor"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["corridor"]->initialise("corridor.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["corridor"]->getRigidBody());

    mInitialised = true;

    return true;
}

void CarCrashSimulation::tick(){
    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        mWorldEntities[mGroupOneAgents[k]]->tick();

    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        mWorldEntities[mGroupTwoAgents[k]]->tick();
}

double CarCrashSimulation::getRayCollisionDistance(string _agentName, const btVector3& _ray){
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

void CarCrashSimulation::applyUpdateRules(string _agentName, uint _groupNum){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    map<uint, double> input;
    //rangefinders
    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 0)) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 0)) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100)) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100)) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -100)) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 100)) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0, -100)) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, 100)) / 50;

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

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName]->update(output);

    bool checkCollisions = _groupNum == 1? calcCrossVal(mGroupOneFinish.p1, mGroupOneFinish.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0 : 
        calcCrossVal(mGroupTwoFinish.p1, mGroupTwoFinish.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) < 0;

    if(checkCollisions){
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
 
vector3 CarCrashSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}
