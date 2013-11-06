#include "agent.h"

Agent::Agent(){}

bool Agent::initialise(const string& _modelName, const vector3& _scale, const btQuaternion& _rotation, ResourceManager* _rm, const vector3& _position, const double& _mass){
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

    return true;
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