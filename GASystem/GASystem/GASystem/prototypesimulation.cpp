#include "prototypesimulation.h"

PrototypeSimulation::PrototypeSimulation(vector<vector3> _waypoints, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(PrototypeSimulation::tickCallBack, this, true);
    mWaypoints = _waypoints;
    mCollisions = 0;
}

PrototypeSimulation::~PrototypeSimulation(){}

void PrototypeSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        applyUpdateRules("agentOne");
        applyUpdateRules("agentTwo");
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double PrototypeSimulation::fitness(vector<Fitness*> _fit){
    double finalFitness = 0;

    for(uint k = 0; k < _fit.size(); k++){
        map<string, vector3> pos;
        pos["agentOne"] = getPositionInfo("agentOne");
        pos["agentTwo"] = getPositionInfo("agentTwo");

        mWaypointTracker["Collisions"] = mCollisions / 6;

        finalFitness += _fit[k]->evaluateFitness(pos, map<string, double>(), mWaypointTracker);
    }

    cout << finalFitness << endl;

    return finalFitness;
}

vector3 PrototypeSimulation::getPositionInfo(string _entityName){
    btRigidBody* rb = mWorldEntities["agentOne"].get<0>();
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
    vector3 maxVel(10, 10, 10), minVel(-10, -10, -10), agentOneVel, agentTwoVel;

    btRigidBody* agentOne = mWorldEntities["agentOne"].get<0>();
    btRigidBody* agentTwo = mWorldEntities["agentTwo"].get<0>();

    bool update = false;

    //agent one conform
    if(agentOne->getLinearVelocity().getX() > maxVel.x)
        agentOneVel.x = maxVel.x;
    else if(agentOne->getLinearVelocity().getX() < minVel.x)
        agentOneVel.x = minVel.x;
    else agentOneVel.x = agentOne->getLinearVelocity().getX();

    if(agentOne->getLinearVelocity().getY() > maxVel.y)
        agentOneVel.y = maxVel.y;
    else if(agentOne->getLinearVelocity().getY() < minVel.y)
        agentOneVel.y = minVel.y;
    else agentOneVel.y = agentOne->getLinearVelocity().getY();

    if(agentOne->getLinearVelocity().getZ() > maxVel.z)
        agentOneVel.z = maxVel.z;
    else if(agentOne->getLinearVelocity().getZ() < minVel.z)
        agentOneVel.z = minVel.z;
    else agentOneVel.z = agentOne->getLinearVelocity().getZ();

    //agent two conform
    if(agentTwo->getLinearVelocity().getX() > maxVel.x)
        agentTwoVel.x = maxVel.x;
    else if(agentTwo->getLinearVelocity().getX() < minVel.x)
        agentTwoVel.x = minVel.x;
    else agentTwoVel.x = agentOne->getLinearVelocity().getX();

    if(agentTwo->getLinearVelocity().getY() > maxVel.y)
        agentTwoVel.y = maxVel.y;
    else if(agentTwo->getLinearVelocity().getY() < minVel.y)
        agentTwoVel.y = minVel.y;
    else agentTwoVel.y = agentTwo->getLinearVelocity().getY();

    if(agentTwo->getLinearVelocity().getZ() > maxVel.z)
        agentTwoVel.z = maxVel.z;
    else if(agentTwo->getLinearVelocity().getZ() < minVel.z)
        agentTwoVel.z = minVel.z;
    else agentTwoVel.z = agentTwo->getLinearVelocity().getZ();

    agentOne->setLinearVelocity(btVector3(agentOneVel.x, agentOneVel.y, agentOneVel.z));
    agentTwo->setLinearVelocity(btVector3(agentTwoVel.x, agentTwoVel.y, agentTwoVel.z));
}

bool PrototypeSimulation::initialise(){
    if(mInitialised)
        return true;

    //agents
    if(!createObject("cube.mesh", "agentOne", vector3(1, 1, 1), vector3(60, -2, 54), 0.01, mResourceManager, false, false))
        return false;

    if(!createObject("cube.mesh", "agentTwo", vector3(1, 1, 1), vector3(60, -2, 50), 0.01, mResourceManager, false, false))
        return false;

    mWaypointTracker["agentOne"] = mWaypointTracker["agentTwo"] = 0;
    
    //maze
    if(!createObject("maze.mesh", "maze", vector3(10, 10, 10), vector3(0, 0, 0), 0, mResourceManager, true, true))
        return false;

    mInitialised = true;
    
    return true;
}

bool PrototypeSimulation::createObject(string _meshname, string _entityName, vector3 _scale, vector3 _position, float _mass, ResourceManager* _rm, bool _static, bool _concave){
    btCollisionShape* shape = _rm->getBulletCollisionShape(_meshname, _static, _concave, vector3(0, 0, 0), _scale);
    
    if(!shape)
        return false;

    btMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(_position.x, _position.y, _position.z)));
    
    btVector3 inertia(0, 0, 0);

    if(!_static)
        shape->calculateLocalInertia(_mass, inertia);
    
    btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

    btRigidBody* rbody = new btRigidBody(constructionInfo);
    rbody->setRestitution(0.5);
    rbody->setSleepingThresholds(0.f, 0.0f);
    mWorld->addRigidBody(rbody);
    mWorldEntities[_entityName] = ObjectInfo(rbody, _meshname, _scale);

    return true;
}

void PrototypeSimulation::applyUpdateRules(string _agentName){
    btTransform trans;
    mWorldEntities[_agentName].get<0>()->getMotionState()->getWorldTransform(trans);

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

    double xAccel = (output[0] - 0.5) * 10;
    double zAccel = (output[1] - 0.5) * 10;
    
    mWorldEntities[_agentName].get<0>()->applyCentralForce(btVector3(xAccel, 0, zAccel));

    btVector3 vel = mWorldEntities[_agentName].get<0>()->getLinearVelocity();

    int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

        if(mWorldEntities[_agentName].get<0>() == obA || mWorldEntities[_agentName].get<0>() == obB)
            mCollisions++;
    }

    //mWorldEntities[_agentName].get<0>()->setLinearVelocity(btVector3(vel.getX() + xAccel, 0, vel.getZ() + zAccel));
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
        _collisionDistances[1] = calcEucDistance(from, to);
    }
    else _collisionDistances[1] = 2000;

    if(botRay.hasHit()){
        vector3 to(botRay.m_hitPointWorld.getX(), botRay.m_hitPointWorld.getY(), botRay.m_hitPointWorld.getZ());
        _collisionDistances[1] = calcEucDistance(from, to);
    }
    else _collisionDistances[2] = 2000;

    if(leftRay.hasHit()){
        vector3 to(leftRay.m_hitPointWorld.getX(), leftRay.m_hitPointWorld.getY(), leftRay.m_hitPointWorld.getZ());
        _collisionDistances[3] = calcEucDistance(from, to);
    }
    else _collisionDistances[3] = 2000;

    if(rightRay.hasHit()){
        vector3 to(rightRay.m_hitPointWorld.getX(), rightRay.m_hitPointWorld.getY(), rightRay.m_hitPointWorld.getZ());
        _collisionDistances[4] = calcEucDistance(from, to);
    }
    else _collisionDistances[4] = 2000;
}

double PrototypeSimulation::calcDistance(vector3 _from, vector3 _to){
    double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;

    return sqrt(x*x + y*y + z*z);
}