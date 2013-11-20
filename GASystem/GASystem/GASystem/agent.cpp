#include "agent.h"

Agent::Agent() : mRNG(0), mDist(0, 1), generateRandInt(mRNG, mDist), mAvoidanceMode(false){}

bool Agent::initialise(const string& _modelName, const vector3& _scale, const btQuaternion& _rotation, ResourceManager* _rm, const vector3& _position, const double& _mass, int _seed){
    mModelName = _modelName;
    mScale = _scale;

    btCollisionShape* shape = getCollisionShape(_rm);
    if(!shape){
        cerr << "Error: unable to get collision shape for model " << _modelName << endl;
        return false;
    }

    btMotionState* ms = new btDefaultMotionState(btTransform(_rotation, btVector3(_position.x, _position.y, _position.z)));

    btVector3 inertia = calculateInertia(_mass, shape);
    
    btRigidBody::btRigidBodyConstructionInfo constructionInfo(_mass, ms, shape, inertia);

    mRigidBody = new btRigidBody(constructionInfo);

    setRigidbodyProperties();

    mRNG = boost::mt19937(_seed);
    mDist = boost::uniform_int<>(0, 10000);
    generateRandInt = boost::variate_generator<boost::mt19937, boost::uniform_int<>>(mRNG, mDist);

    return true;
}

void Agent::avoided(){
    mAvoidanceMode = false;
}

Agent::~Agent(){
    delete mRigidBody->getCollisionShape();
    delete mRigidBody->getMotionState();
    delete mRigidBody;
}

btRigidBody* Agent::getRigidBody(){
    return mRigidBody;
}

string Agent::getModelName(){
    return mModelName;
}

vector3 Agent::getScale(){
    return mScale;
}

double Agent::getRayCollisionDistance(const btVector3& _ray, btDiscreteDynamicsWorld* _world){
    double dist = 100;

    btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_ray, _world);

    btTransform trans;
    mRigidBody->getMotionState()->getWorldTransform(trans);
    vector3 from(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());

    if(ray.hasHit())
        dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));

    return dist;
}

btCollisionWorld::ClosestRayResultCallback Agent::calculateRay(const btVector3& _ray, btDiscreteDynamicsWorld* _world){
    btVector3 correctedRot = mRigidBody->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mRigidBody->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

    _world->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

void Agent::setVelocity(vector3 _velocity){
    mRigidBody->setLinearVelocity(btVector3(_velocity.x, _velocity.y, _velocity.z));
}