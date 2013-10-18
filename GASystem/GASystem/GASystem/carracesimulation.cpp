#include "carracesimulation.h"

CarRaceSimulation::CarRaceSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
: Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CarRaceSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < 10; ++k)
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));

    mWinToExpectedDistance = 0;
}

CarRaceSimulation::~CarRaceSimulation(){
    
}

void CarRaceSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        
        for(uint k = 1; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k], 1);
        applyUpdateRules(mAgents[0], 0);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CarRaceSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    intAcc["Collisions"] = ceil(mRangefinderVals) + mCollisions; 
    intAcc["FLFitnessWeight"] = 1;
    intAcc["ColFitnessWeight"] = 1;
    intAcc["WinnerFitnessWeight"] = 1;
    
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);
    intAcc["Positive"] = 0;
    intAcc["Winner"] = mWinner;
    intAcc["ExpectedWinner"] = 0;
    intAcc["WinnerVal"] = ceil(mWinToExpectedDistance);
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, map<string, double>(), intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc) : 1000;
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, map<string, double>(), intAcc) : 1000;

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

    mWorldEntities[mAgents[0]] = new CarAgent(10, 1);
    if(!mWorldEntities[mAgents[0]]->initialise("carone.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(4.5, 0, 38), 0.01))
        return false;
    mWorld->addRigidBody(mWorldEntities[mAgents[0]]->getRigidBody());

    for(uint k = 1; k < mAgents.size(); k++){
        vector3 pos;
        if(k < 6){
            pos.x = ((double)k - 1) * 3 - 7.5;
            pos.y = 0;
            pos.z = 34;
        }
        else{
            pos.x = ((double)k - 6) * 3 - 7.5;
            pos.y = 0;
            pos.z = 38;
        }

        mWorldEntities[mAgents[k]] = new CarAgent(10, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, pos, 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["racetrack"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["racetrack"]->initialise("racetrack.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 3, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["racetrack"]->getRigidBody());

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
    intAcc["Collisions"] = mCollisions; 
    intAcc["FLFitnessWeight"] = 1;
    intAcc["ColFitnessWeight"] = 1;
    intAcc["WinnerFitnessWeight"] = 1;
    
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);
    intAcc["Positive"] = 0;
    intAcc["Winner"] = mWinner;
    intAcc["ExpectedWinner"] = 0;
    intAcc["WinnerVal"] = mWinToExpectedDistance;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, map<string, double>(), intAcc);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), intAcc) : 1000;
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, map<string, double>(), intAcc) : 1000;

    return finalFitness;
}

double CarRaceSimulation::getRayCollisionDistance(string _agentName, const btVector3& _ray){
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

void CarRaceSimulation::applyUpdateRules(string _agentName, uint _groupNum){
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
    input[11] = _groupNum == mFinishLine.p1.x / 50;
    input[12] = _groupNum == mFinishLine.p1.z / 50;
    input[13] = _groupNum == mFinishLine.p2.x / 50;
    input[14] = _groupNum == mFinishLine.p2.z / 50;

    //velocity
    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[15] = agentVel.x;
    input[16] = agentVel.z;

    if(_groupNum == 1){
        input[17] = mWorldEntities[mAgents[0]]->getVelocity().x;
        input[18] = mWorldEntities[mAgents[0]]->getVelocity().z;

        btTransform winnerTrans;
        mWorldEntities[mAgents[0]]->getRigidBody()->getMotionState()->getWorldTransform(winnerTrans);
        input[19] = winnerTrans.getOrigin().getX() / 50;
        input[20] = winnerTrans.getOrigin().getZ() / 50;
    }



    vector<double> output = mSolution->evaluateNeuralNetwork(_groupNum, input);

    mWorldEntities[_agentName]->update(output);

    bool checkCollisions = calcCrossVal(mFinishLine.p1, mFinishLine.p2, vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())) > 0;

    if(!checkCollisions && mWinner == -1){
        for(uint k = 0; k < mAgents.size(); k++)
            if(mAgents[k] == _agentName){
                btTransform winnerTrans;
                mWorldEntities[mAgents[0]]->getRigidBody()->getMotionState()->getWorldTransform(winnerTrans);
           
                mWinner = k;
                mWinToExpectedDistance = vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()).calcDistance(vector3(winnerTrans.getOrigin().getX(), winnerTrans.getOrigin().getY(), winnerTrans.getOrigin().getZ()));
                break;
            }
    }

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

vector3 CarRaceSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}