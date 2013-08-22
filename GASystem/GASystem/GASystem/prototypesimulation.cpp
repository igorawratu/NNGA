#include "prototypesimulation.h"

PrototypeSimulation::PrototypeSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution){
}

PrototypeSimulation::~PrototypeSimulation(){}

void PrototypeSimulation::iterate(){
    //input logic here

    mWorld->stepSimulation(1.f/mCyclesPerSecond, 1, 1.f/mCyclesPerSecond);
}

double PrototypeSimulation::fitness(vector<Fitness*> _fit){
    //redefine this
    double finalFitness = 0;
    map<uint, double> dblAcc;
    map<uint, long> intAcc;
    vector<vector3> pos;

    for(uint k = 0; k < _fit.size(); k++)
        finalFitness += _fit[k]->evaluateFitness(pos, dblAcc, intAcc);

    return finalFitness;
}

Simulation* PrototypeSimulation::getNewCopy(){
    return new PrototypeSimulation(mNumCycles, mCyclesPerDecision, mCyclesPerDecision, mSolution);
}

bool PrototypeSimulation::initialise(ResourceManager* _rm){
    if(mInitialised)
        return true;

    //agents
    createRectangularObject("cube.mesh", "agentOne", vector3(1, 1, 1), vector3(0, 10, -10), 1, _rm);
    createRectangularObject("cube.mesh", "agentTwo", vector3(1, 1, 1), vector3(0, -10, -10), 1, _rm);
    
    //create maze here

    mInitialised = true;
    
    return true;
}

void PrototypeSimulation::createRectangularObject(string _meshname, string _entityName, vector3 _scale, vector3 _position, float _mass, ResourceManager* _rm){
    btConvexShape* shape = _rm->getBulletCollisionShape(_meshname, vector3(0, 0, 0), Ogre::Quaternion::IDENTITY, _scale);
    btMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(_position.x, _position.y, _position.z)));
    
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(_mass, inertia);
    
    btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

    btRigidBody* rbody = new btRigidBody(constructionInfo);
    mWorld->addRigidBody(rbody);
    mWorldEntities[_entityName] = make_pair(rbody, _meshname);
}