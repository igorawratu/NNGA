#include "corneringsim.h"

CorneringSim::CorneringSim(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderVals = 0;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < _numAgents; k++){
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
        mWaypointTracker[mAgents[k]] = 0;
    }
}   

CorneringSim::~CorneringSim(){

}

CorneringSim::CorneringSim(const CorneringSim& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = other.mSeed;
    mRangefinderVals = 0;
    mRangefinderRadius = other.mRangefinderRadius;

    for(uint k = 0; k < other.mAgents.size(); k++){
        mAgents.push_back("Agent" + boost::lexical_cast<string>(k));
        mWaypointTracker[mAgents[k]] = 0;
    }
}   

void CorneringSim::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(int k = 0; k < mAgents.size(); k++){
            applyUpdateRules(mAgents[k]);
            //calcCollisions(mAgents[k]);
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CorneringSim::fitness(){
    double finalFitness = 0;
    double maxCollisions = (mNumCycles / mCyclesPerDecision) * mAgents.size() * 9;

    map<string, vector3> pos;
    map<string, double> doubleAcc;
    map<string, long> intAcc;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["Collisions"] = mCollisions + mRangefinderVals;
    doubleAcc["WPFitnessWeight"] = 1;
    intAcc = mWaypointTracker;

    for(uint k = 0; k < mWaypoints.size(); k++)
        pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k];

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    
    finalFitness += 2 * mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += /*finalFitness == 0 ? */mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, mWaypointTracker)/* : maxCollisions*/;

    return finalFitness;
}

double CorneringSim::realFitness(){
    double finalFitness = 0;
    double maxCollisions = (mNumCycles / mCyclesPerDecision) * mAgents.size() * 9;

    map<string, vector3> pos;
    map<string, double> doubleAcc;
    map<string, long> intAcc;
    doubleAcc["ColFitnessWeight"] = 1;
    doubleAcc["Collisions"] = mCollisions;
    doubleAcc["WPFitnessWeight"] = 1;
    intAcc = mWaypointTracker;

    for(uint k = 0; k < mWaypoints.size(); k++)
        pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k];

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    
    finalFitness += 2 * mFitnessFunctions[0]->evaluateFitness(pos, doubleAcc, intAcc);
    finalFitness += /*finalFitness == 0 ? */mFitnessFunctions[1]->evaluateFitness(pos, doubleAcc, mWaypointTracker)/* : maxCollisions*/;

    return finalFitness;
}

Simulation* CorneringSim::getNewCopy(){
    Simulation* sim = new CorneringSim(*this);
    sim->initialise();

    return sim;
}

bool CorneringSim::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new WaypointFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mWaypointTracker["ColFitnessWeight"] = 1;
    mWaypointTracker["NumAgents"] = mAgents.size();

    mWaypoints.push_back(vector3(-35, 0, -40));
    mWaypoints.push_back(vector3(5, 0, -30));
    mWaypoints.push_back(vector3(25, 0, 30));

    mWaypointTracker["NumWaypoints"] = mWaypoints.size();

    //set the vals
    vector3 minDim(-45, 0, 25), maxDim(-40, 0, 35);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);
    rot.setEuler(PI/2, 0, 0);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new CarAgent(15, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), minDim.y, genz()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("corneringtrack.mesh", vector3(50, 5, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    /*for(uint k = 0; k < mWaypoints.size(); k++){
        mWorldEntities["waypoint" + boost::lexical_cast<string>(k)] = new StaticWorldAgent(0.5, 0.1);
        if(!mWorldEntities["waypoint" + boost::lexical_cast<string>(k)]->initialise("sphere.mesh", vector3(5, 5, 5), btQuaternion(0, 0, 0, 1), mResourceManager, mWaypoints[k], 0, mSeed))
            return false;
    }*/

    mInitialised = true;
    
    return true;
}

void CorneringSim::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    double frontVal = -1;

    if(mWaypointTracker[_agentName] < mWaypoints.size()){
        if(vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()).calcDistance(mWaypoints[mWaypointTracker[_agentName]]) < 5)
            mWaypointTracker[_agentName]++;
    }

    map<uint, double> input;

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));

    double frontDist = d1 > d2 ? d2 : d1;

    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0), AGENT) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 0), AGENT) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, 100), AGENT) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, -100), AGENT) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -100), AGENT) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 100), AGENT) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, -100), AGENT) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 100), AGENT) / 50;

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[9] = agentVel.x;
    input[10] = agentVel.z;

    input[11] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].x;
    input[12] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].z;

    //agent position
    input[13] = trans.getOrigin().getX() / 50;
    input[14] = trans.getOrigin().getZ() / 50;

    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(frontDist, mCyclesPerSecond, mCyclesPerDecision, mWorld);
    else{
        mWorldEntities[_agentName]->avoided();
        vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
        mWorldEntities[_agentName]->update(output);
    }

    if(mWaypointTracker[_agentName] < mWaypoints.size() && mCycleCounter > 10){
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

void CorneringSim::tick(){
    for(uint k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}
