#ifndef MOUSEAGENT_H
#define MOUSEAGENT_H

#include "agent.h"

#include <iostream>

using namespace std;

enum RotationType{NOROTATION, RIGHT, LEFT};

class MouseAgent : public Agent
{
public:
    MouseAgent(double _walkSpeed, double _runSpeed, double _rotationAngle){
        mWalkSpeed = _walkSpeed;
        mRunSpeed = _runSpeed;
        mRotationAngle = _rotationAngle;
        mRotationType = NOROTATION;
    }

    virtual void update(const vector<double>& _nnOutput){
        assert(_nnOutput.size() >= 2);
        //assumes output between 0 and 1

        //rotation
        if(_nnOutput[0] < 0.3){
            mRotationType = LEFT;
            mCurrRotAngle = 0;
        }
        else if(_nnOutput[0] >= 0.3 && _nnOutput[0] < 0.7){
            //do nothing
            mRotationType = RIGHT;
            mCurrRotAngle = 0;
        }
        else{
            mRotationType = RIGHT;
            mCurrRotAngle = 0;
        }

        //velocity
        if(_nnOutput[1] < 0.3)
            mRigidBody->setLinearVelocity(btVector3(0, 0, 0));
        else{
            btVector3 relativeVel;
            if(_nnOutput[0] >= 0.3 && _nnOutput[0] < 0.7)
                relativeVel = btVector3(mWalkSpeed, 0, 0);
            else
                relativeVel = btVector3(mRunSpeed, 0, 0);

            btMatrix3x3& rot = mRigidBody->getWorldTransform().getBasis();
            btVector3 correctedVel = rot * relativeVel;
            mRigidBody->setLinearVelocity(-correctedVel);
        }


    }

    virtual void tick(){
        btQuaternion quat;
        double rot = 0;
        if(mRotationType == RIGHT){
            rot = mCurrRotAngle - 0.1 < -mRotationAngle ? - mRotationAngle - mCurrRotAngle : -0.1;
            mCurrRotAngle += rot;
            mRotationType = mCurrRotAngle == -mRotationAngle ? NOROTATION : RIGHT;
        }
        else if(mRotationType == LEFT){
            rot = mCurrRotAngle + 0.1 > mRotationAngle ? mRotationAngle - mCurrRotAngle : 0.1;
            mCurrRotAngle += rot;
            mRotationType = mCurrRotAngle == mRotationAngle ? NOROTATION : LEFT;
        }

        cout << rot << endl;

        mRigidBody->getWorldTransform().setRotation(mRigidBody->getWorldTransform().getRotation() * btQuaternion(btVector3(0, 1, 0), rot));
        if(mRotationType == NOROTATION)
            mCurrRotAngle = 0;
    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){
        return _rm->getBulletCollisionShape(mModelName, false, false, vector3(0, 0, 0), mScale);
    }

    virtual void setRigidbodyProperties(){
        mRigidBody->setRestitution(0.1);
        mRigidBody->setSleepingThresholds(0.f, 0.0f);
    }

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape){
        btVector3 inertia(0, 0, 0);
        _shape->calculateLocalInertia(_mass, inertia);

        return inertia;
    }

private:
    double mWalkSpeed;
    double mRunSpeed;
    double mRotationAngle;
    double mCurrRotAngle;
    RotationType mRotationType;
};

#endif