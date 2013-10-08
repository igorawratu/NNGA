#include "corneringsim.h"

CorneringSim::CorneringSim(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mRangefinderVals = 0;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < _numAgents; k++){
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));
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
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));
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

    mWaypointTracker["Collisions"] = mCollisions + mRangefinderVals / 5;

    for(uint k = 0; k < mWaypoints.size(); k++)
        pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k];

    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    
    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, map<string, double>(), mWaypointTracker);
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, map<string, double>(), mWaypointTracker) : maxCollisions;

    cout << mWaypointTracker["Collisions"] << "  " << finalFitness << endl;

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

    mWaypointTracker["WPFitnessWeight"] = 1;
    mWaypointTracker["ColFitnessWeight"] = 1;
    mWaypointTracker["NumAgents"] = mAgents.size();

    mWaypoints.push_back(vector3(43, 0, -20));
    mWaypoints.push_back(vector3(41, 0, -8));
    mWaypoints.push_back(vector3(-22.5, 0, 10));
    mWaypoints.push_back(vector3(36, 0, 24));
    mWaypoints.push_back(vector3(37, 0, 44));

    mWaypointTracker["NumWaypoints"] = mWaypoints.size();

    //set the vals
    vector3 minDim(-45, 0, -45), maxDim(-25, 0, -35);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    btQuaternion rot(0, 0, 0, 1);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new CarAgent(10, 1);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rot, mResourceManager, vector3(genx(), minDim.y, genz()), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["corneringtrack"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["corneringtrack"]->initialise("corneringtrack.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 6.5, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["corneringtrack"]->getRigidBody());

    /*for(uint k = 0; k < mWaypoints.size(); k++){
        mWorldEntities["waypoint" + boost::lexical_cast<string>(k)] = new StaticWorldAgent(0.5, 0.1);
        if(!mWorldEntities["waypoint" + boost::lexical_cast<string>(k)]->initialise("sphere.mesh", vector3(3, 3, 3), btQuaternion(0, 0, 0, 1), mResourceManager, mWaypoints[k], 0))
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

    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0)) / 50;
    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 0)) / 50;
    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, 100)) / 50;
    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0.1, -100)) / 50;
    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, -100)) / 50;
    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, 100)) / 50;
    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0.1, -100)) / 50;
    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 100)) / 50;
    frontVal = getRayCollisionDistance(_agentName, btVector3(100, 0.1, 0));

    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[9] = agentVel.x;
    input[10] = agentVel.z;

    input[11] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].x;
    input[12] = mWaypoints[mWaypointTracker[_agentName] < mWaypoints.size() ? mWaypointTracker[_agentName] : mWaypoints.size() - 1].z;

    //agent position
    input[13] = trans.getOrigin().getX() / 50;
    input[14] = trans.getOrigin().getZ() / 50;

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
    output.push_back(frontVal);

    mWorldEntities[_agentName]->update(output);

    if(mWaypointTracker[_agentName] < mWaypoints.size()){
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

double CorneringSim::getRayCollisionDistance(string _agentName, const btVector3& _ray){
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

vector3 CorneringSim::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}
