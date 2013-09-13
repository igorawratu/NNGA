#include "corneringsim.h"

CorneringSim::CorneringSim(vector<vector3> _waypoints, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CorneringSim::tickCallBack, this, true);
    mCollisions = 0;
    mSeed = _seed;
    mWaypoints = _waypoints;

    for(uint k = 0; k < _numAgents; k++){
        mAgents.push_back("agent" + boost::lexical_cast<string>(k));
        mWaypointTracker[mAgents[k]] = 0;
    }
}   

CorneringSim::~CorneringSim(){

}

void CorneringSim::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        for(uint k = 0; k < mAgents.size(); k++)
            applyUpdateRules(mAgents[k]);
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double CorneringSim::fitness(vector<Fitness*> _fit){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;

    intAcc["Collisions"] = mCollisions;
    for(uint k = 0; k < mAgents.size(); k++)
        pos[mAgents[k]] = getPositionInfo(mAgents[k]);

    for(uint k = 0; k < _fit.size(); k++)
        finalFitness += _fit[k]->evaluateFitness(pos, map<string, double>(), intAcc);

    cout << finalFitness << endl;

    return finalFitness;
}

Simulation* CorneringSim::getNewCopy(){
    Simulation* sim = new CorneringSim(mWaypoints, mAgents.size(), mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    sim->initialise();

    return sim;
}

bool CorneringSim::initialise(){
    if(mInitialised)
        return true;

    //set the vals
    vector3 minDim(-1, -1, -1), maxDim(1, 1, 1);

    boost::mt19937 rng(mSeed);
    boost::mt19937 rngz(mSeed + mSeed / 2);

    boost::uniform_real<double> distx(minDim.x, maxDim.x);
    boost::uniform_real<double> disty(minDim.y, maxDim.y);
    boost::uniform_real<double> distz(minDim.z, maxDim.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genx(rng, distx);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> geny(rng, disty);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genz(rngz, distz);

    for(uint k = 0; k < mAgents.size(); k++){
        mWorldEntities[mAgents[k]] = new CarAgent(5, 0.25);
        if(!mWorldEntities[mAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(genx(), geny(), genz()), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["corneringtrack"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["corneringtrack"]->initialise("corneringtrack.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["corneringtrack"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void CorneringSim::applyUpdateRules(string _agentName){
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
    input[10] = trans.getOrigin().getY() / 50;
    input[11] = trans.getOrigin().getZ() / 50;
    
    //current waypoint
    vector3 currWaypoint;
    if(mWaypointTracker[_agentName] < mWaypoints.size()){
        if(calcDistance(vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()), mWaypoints[mWaypointTracker[_agentName]]) < 10)
            mWaypointTracker[_agentName]++;
    }
    currWaypoint = mWaypointTracker[_agentName] >= mWaypoints.size() ? mWaypoints[mWaypoints.size() - 1] : mWaypoints[mWaypointTracker[_agentName]];

    input[12] = currWaypoint.x / 50;
    input[13] = currWaypoint.y / 50;
    input[14] = currWaypoint.z / 50;

    vector<double> output = mSolution->evaluateNeuralNetwork(0, input);

    mWorldEntities[_agentName]->update(output);

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

void CorneringSim::tick(){
    for(uint k = 0; k < mAgents.size(); k++)
        mWorldEntities[mAgents[k]]->tick();
}

double CorneringSim::getRayCollisionDistance(string _agentName, const btVector3& _ray){
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

double CorneringSim::calcDistance(vector3 _from, vector3 _to){
    double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

    return sqrt(x*x + y*y + z*z);
}

vector3 CorneringSim::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}
