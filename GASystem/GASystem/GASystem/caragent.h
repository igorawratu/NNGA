#ifndef CARAGENT_H
#define CARAGENT_H

#include "agent.h"

class CarAgent : public Agent
{
public:
    CarAgent(){
    }

    virtual void update(const vector<double>& _nnOutput){

    }

protected:
    virtual btCollisionShape* getCollisionShape(ResourceManager* _rm){

    }

    virtual void setRigidbodyProperties(){

    }

    virtual btVector3 calculateInertia(double _mass, btCollisionShape* _shape){

    }
};

#endif