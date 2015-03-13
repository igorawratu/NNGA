#include "prototypesimulation.h"

PrototypeSimulation::PrototypeSimulation(vector<vector3> _waypoints, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager, TeamSetup::HET){
    mWorld->setInternalTickCallback(PrototypeSimulation::tickCallBack, this, true);
    mWaypoints = _waypoints;
    mCollisions = 0;
}

PrototypeSimulation::~PrototypeSimulation(){}

void PrototypeSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        applyUpdateRules("Agent0");
        applyUpdateRules("Agent1");
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double PrototypeSimulation::fitness(){
    double finalFitness = 0;

    for(uint k = 0; k < mFitnessFunctions.size(); k++){
        map<string, vector3> pos;
        pos["Agent0"] = getPositionInfo("Agent0");
        pos["Agent1"] = getPositionInfo("Agent1");
        for(uint k = 1; k <= mWaypoints.size(); k++)
            pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k - 1];

        mWaypointTracker["Collisions"] = mCollisions / 6;

        finalFitness += mFitnessFunctions[k]->evaluateFitness(pos, map<string, double>(), mWaypointTracker);
    }

    cout << finalFitness << endl;

    return finalFitness;
}

double PrototypeSimulation::realFitness(){
    double finalFitness = 0;

    for(uint k = 0; k < mFitnessFunctions.size(); k++){
        map<string, vector3> pos;
        pos["Agent0"] = getPositionInfo("Agent0");
        pos["Agent1"] = getPositionInfo("Agent1");
        for(uint k = 1; k <= mWaypoints.size(); k++)
            pos["Waypoint" + boost::lexical_cast<string>(k)] = mWaypoints[k - 1];

        mWaypointTracker["Collisions"] = mCollisions / 6;

        finalFitness += mFitnessFunctions[k]->evaluateFitness(pos, map<string, double>(), mWaypointTracker);
    }

    cout << finalFitness << endl;

    return finalFitness;
}

vector3 PrototypeSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
    btTransform trans;
    rb->getMotionState()->getWorldTransform(trans);

    return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}

Simulation* PrototypeSimulation::getNewCopy(){
    Simulation* newsim = new PrototypeSimulation(mWaypoints, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager);
    newsim->initialise();

    return newsim;
}

void PrototypeSimulation::conformVelocities(){
    mWorldEntities["Agent0"]->tick();
    mWorldEntities["Agent1"]->tick();
}

bool PrototypeSimulation::initialise(){
    if(mInitialised)
        return true;

    mWaypointTracker["NumAgents"] = 2;

    //agents
    mWorldEntities["Agent0"] = new CubeAgent(vector3(10, 10, 10), vector3(-10, -10, -10));
    if(!mWorldEntities["Agent0"]->initialise("car.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(60, -2, 54), 0.01, 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["agent0"]->getRigidBody());
    
    mWorldEntities["Agent1"] = new CubeAgent(vector3(10, 10, 10), vector3(-10, -10, -10));
    if(!mWorldEntities["Agent1"]->initialise("cube.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(60, -2, 50), 0.01, 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["agent1"]->getRigidBody());

    mWaypointTracker["Agent0"] = mWaypointTracker["Agent1"] = 0;
    
    //maze
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("maze.mesh", vector3(10, 10, 10), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, 0))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;
    
    return true;
}

void PrototypeSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

    map<uint, double> input;
    map<uint, double> repulsionFactor;
    //rangefinders
    getRayCollisionDistances(input, trans.getOrigin());
    input[1] /= 50;
    input[2] /= 50;
    input[3] /= 50;
    input[4] /= 50;

    //agent position
    input[5] = trans.getOrigin().getX() / 25;
    input[6] = trans.getOrigin().getY() / 25;
    input[7] = trans.getOrigin().getZ() / 25;
    
    //agent current waypoint position
    vector3 currWaypoint;
    if(mWaypointTracker[_agentName] < mWaypoints.size()){
        if(calcDistance(vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()), mWaypoints[mWaypointTracker[_agentName]]) < 10)
            mWaypointTracker[_agentName]++;
    }
    currWaypoint = mWaypointTracker[_agentName] >= mWaypoints.size() ? mWaypoints[mWaypoints.size() - 1] : mWaypoints[mWaypointTracker[_agentName]];

    input[8] = currWaypoint.x / 25;
    input[9] = currWaypoint.y / 25;
    input[10] = currWaypoint.z / 25;

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

void PrototypeSimulation::getRayCollisionDistances(map<uint, double>& _collisionDistances, const btVector3& _agentPosition){
    btVector3 toTop(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ() + 2000);
    btVector3 toLeft(_agentPosition.getX() - 2000, _agentPosition.getY(), _agentPosition.getZ());
    btVector3 toRight(_agentPosition.getX() + 2000, _agentPosition.getY(), _agentPosition.getZ());
    btVector3 toBot(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ() - 2000);

    btCollisionWorld::ClosestRayResultCallback topRay(_agentPosition, toTop);
    btCollisionWorld::ClosestRayResultCallback leftRay(_agentPosition, toLeft);
    btCollisionWorld::ClosestRayResultCallback rightRay(_agentPosition, toRight);
    btCollisionWorld::ClosestRayResultCallback botRay(_agentPosition, toBot);

    mWorld->rayTest(_agentPosition, toTop, topRay);
    mWorld->rayTest(_agentPosition, toBot, botRay);
    mWorld->rayTest(_agentPosition, toLeft, leftRay);
    mWorld->rayTest(_agentPosition, toRight, rightRay);

    vector3 from(_agentPosition.getX(), _agentPosition.getY(), _agentPosition.getZ());
    if(topRay.hasHit()){
        vector3 to(topRay.m_hitPointWorld.getX(), topRay.m_hitPointWorld.getY(), topRay.m_hitPointWorld.getZ());
        _collisionDistances[1] = from.calcDistance(to);
    }
    else _collisionDistances[1] = 2000;

    if(botRay.hasHit()){
        vector3 to(botRay.m_hitPointWorld.getX(), botRay.m_hitPointWorld.getY(), botRay.m_hitPointWorld.getZ());
        _collisionDistances[1] = from.calcDistance(to);
    }
    else _collisionDistances[2] = 2000;

    if(leftRay.hasHit()){
        vector3 to(leftRay.m_hitPointWorld.getX(), leftRay.m_hitPointWorld.getY(), leftRay.m_hitPointWorld.getZ());
        _collisionDistances[3] = from.calcDistance(to);
    }
    else _collisionDistances[3] = 2000;

    if(rightRay.hasHit()){
        vector3 to(rightRay.m_hitPointWorld.getX(), rightRay.m_hitPointWorld.getY(), rightRay.m_hitPointWorld.getZ());
        _collisionDistances[4] = from.calcDistance(to);
    }
    else _collisionDistances[4] = 2000;
}

double PrototypeSimulation::calcDistance(vector3 _from, vector3 _to){
    double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

    return sqrt(x*x + y*y + z*z);
}

ESPParameters PrototypeSimulation::getESPParams(string _nnFormatFile){
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

StandardGAParameters PrototypeSimulation::getSGAParameters(string _nnFormatFile){
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

CMAESParameters PrototypeSimulation::getCMAESParameters(string _nnFormatFile){
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