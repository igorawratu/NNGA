#include "agent.h"

Agent::Agent() : mRNG(0), mDist(0, 1), generateRandInt(mRNG, mDist), mAvoidanceMode(false){
}

bool Agent::initialise(const string& _modelName, const vector3& _scale, const btQuaternion& _rotation, ResourceManager* _rm, const vector3& _position, const double& _mass, int _seed){
    mAnimationName = mLastAnimation = "";
    mAnimationLoop = true;

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

string Agent::getAnimationName(){
    return mAnimationName;
}

bool Agent::getAnimationLoop(){
    return mAnimationLoop;
}

string Agent::getLastAnimation(){
    return mLastAnimation;
}

void Agent::setAnimationInfo(string _animationName, bool _loop){
    mLastAnimation = mAnimationName;
    mAnimationName = _animationName;
    mAnimationLoop = _loop;
}   

double Agent::getRayCollisionDistance(const btVector3& _ray, btDiscreteDynamicsWorld* _world, btRigidBody* _envRigidBody){
    double dist = 100;

    btTransform trans;
    mRigidBody->getMotionState()->getWorldTransform(trans);
    vector3 from(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());

    btCollisionWorld::AllHitsRayResultCallback ray = calculateRay(_ray, _world);
    vector<double> hitDistances;
    for(uint k = 0; k < ray.m_collisionObjects.size(); ++k){
        if(ray.m_collisionObjects[k] == _envRigidBody){
            btVector3 hitpoint = ray.m_hitPointWorld[k];
            double newHitDistance = from.calcDistance(vector3(hitpoint.getX(), hitpoint.getY(), hitpoint.getZ()));
            hitDistances.push_back(newHitDistance);
        }
    }

    for(uint k = 0; k < hitDistances.size(); ++k){
        dist = dist > hitDistances[k] ? hitDistances[k] : dist;
    }

    return dist;
}

btCollisionWorld::AllHitsRayResultCallback Agent::calculateRay(const btVector3& _ray, btDiscreteDynamicsWorld* _world){
    btVector3 correctedRot = mRigidBody->getWorldTransform().getBasis() * _ray;

    btTransform trans;
    mRigidBody->getMotionState()->getWorldTransform(trans);

    btVector3 agentPosition = trans.getOrigin();

    btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

    btCollisionWorld::AllHitsRayResultCallback ray(agentPosition, correctedRay);

    _world->rayTest(agentPosition, correctedRay, ray);

    return ray;
}

void Agent::setVelocity(vector3 _velocity){
    mRigidBody->setLinearVelocity(btVector3(_velocity.x, _velocity.y, _velocity.z));
}

vector3 Agent::getAngularVelocity(){
    btVector3 angVel = mRigidBody->getAngularVelocity();

    return vector3(angVel.getX(), angVel.getY(), angVel.getZ());
}