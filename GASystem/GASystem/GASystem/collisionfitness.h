#ifndef COLLISIONFITNESS_H
#define COLLISIONFITNESS_H

#include "fitness.h"

class CollisionFitness : public Fitness
{
public:
    CollisionFitness(){}

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        if(_intAcc.find("Collisions") != _intAcc.end() && _intAcc.find("ColFitnessWeight") != _intAcc.end())
            return _intAcc["Collisions"] * _intAcc["ColFitnessWeight"];
        else return 0;
    }
};

#endif