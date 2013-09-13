#include "carcrashsimulation.h"

CarCrashSimulation::CarCrashSimulation(uint _agentsPerSide, Line _groupOneFinish, Line _groupTwoFinish, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
 : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(CarCrashSimulation::tickCallBack, this, true);
    mCollisions = 0;
    mGroupOneFinish = _groupOneFinish;
    mGroupTwoFinish = _groupTwoFinish;
    mSeed = _seed;

    for(uint k = 0; k < _agentsPerSide; k++){
        mGroupOneAgents.push_back("Group1Agent" + boost::lexical_cast<string>(k));
        mGroupTwoAgents.push_back("Group2Agent" + boost::lexical_cast<string>(k));
    }
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

double CarCrashSimulation::fitness(vector<Fitness*> _fit){
    double finalFitness = 0;

    map<string, vector3> groupOnePos, groupTwoPos;
    map<string, long> intAcc;
    intAcc["Collisions"] = mCollisions / 5;

    for(uint k = 0; k < mGroupOneAgents.size(); k++)
        groupOnePos[mGroupOneAgents[k]] = getPositionInfo(mGroupOneAgents[k]);
    for(uint k = 0; k < mGroupTwoAgents.size(); k++)
        groupTwoPos[mGroupTwoAgents[k]] = getPositionInfo(mGroupTwoAgents[k]);

    for(uint k = 0; k < _fit.size(); k++){
        //check type in order to evaluate separate groups of agents
        finalFitness += _fit[k]->evaluateFitness(groupOnePos, map<string, double>(), intAcc);
    }

    cout << finalFitness << endl;

    return finalFitness;
}

Simulation* CarCrashSimulation::getNewCopy(){
    Simulation* sim = new CarCrashSimulation(mGroupOneAgents.size(), mGroupOneFinish, mGroupTwoFinish, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    sim->initialise();
    
    return sim;
}

bool CarCrashSimulation::initialise(){
    if(mInitialised)
        return true;

    //set the vals
    vector3 minDimOne(-1, 0, 1), maxDimOne(1, 0, 1), minDimTwo(-1, 0, 1), maxDimTwo(1, 0, 1);

    boost::mt19937 rngxOne(mSeed);
    boost::mt19937 rngzOne(mSeed + mSeed / 2);

    boost::uniform_real<double> distxOne(minDimOne.x, maxDimOne.x);
    boost::uniform_real<double> distzOne(minDimOne.z, maxDimOne.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxOne(rngxOne, distxOne);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genzOne(rngzOne, distzOne);

    boost::mt19937 rngxTwo(mSeed / 2);
    boost::mt19937 rngzTwo(mSeed + mSeed / 3);

    boost::uniform_real<double> distxTwo(minDimTwo.x, maxDimTwo.x);
    boost::uniform_real<double> distzTwo(minDimTwo.z, maxDimTwo.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxTwo(rngxTwo, distxTwo);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genzTwo(rngzTwo, distzTwo);


    btQuaternion rotGroupOne(0, 0, 0, 1), rotGroupTwo(0, 0, 0, 1);
    rotGroupOne.setEuler(PI/2, 0, 0);
    rotGroupTwo.setEuler(PI + PI/2, 0, 0);

    for(uint k = 0; k < mGroupOneAgents.size(); k++){
        mWorldEntities[mGroupOneAgents[k]] = new CarAgent(10, 0.5);
        if(!mWorldEntities[mGroupOneAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupOne, mResourceManager, vector3(genxOne(), 0, genzOne()), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupOneAgents[k]]->getRigidBody());
    }

    for(uint k = 0; k < mGroupTwoAgents.size(); k++){
        mWorldEntities[mGroupTwoAgents[k]] = new CarAgent(10, 0.5);
        if(!mWorldEntities[mGroupTwoAgents[k]]->initialise("car.mesh", vector3(1, 1, 1), rotGroupTwo, mResourceManager, vector3(genxTwo(), 0, genzTwo()), 0.01))
            return false;
        mWorld->addRigidBody(mWorldEntities[mGroupTwoAgents[k]]->getRigidBody());
    }
    
    mWorldEntities["alley"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["alley"]->initialise("alley.mesh", vector3(50, 50, 50), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["alley"]->getRigidBody());

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
    input[10] = trans.getOrigin().getY() / 50;
    input[11] = trans.getOrigin().getZ() / 50;
    
    //goal line
    input[12] = _groupNum == 1 ? mGroupOneFinish.p1.x / 50 : mGroupTwoFinish.p1.x / 50;
    input[13] = _groupNum == 1 ? mGroupOneFinish.p1.y / 50 : mGroupTwoFinish.p1.y / 50;
    input[14] = _groupNum == 1 ? mGroupOneFinish.p1.z / 50 : mGroupTwoFinish.p1.z / 50;
    input[15] = _groupNum == 1 ? mGroupOneFinish.p2.x / 50 : mGroupTwoFinish.p2.x / 50;
    input[16] = _groupNum == 1 ? mGroupOneFinish.p2.y / 50 : mGroupTwoFinish.p2.y / 50;
    input[17] = _groupNum == 1 ? mGroupOneFinish.p2.z / 50 : mGroupTwoFinish.p2.z / 50;

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

double CarCrashSimulation::calcDistance(vector3 _from, vector3 _to){
    double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

    return sqrt(x*x + y*y + z*z);
}
 
vector3 CarCrashSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}
