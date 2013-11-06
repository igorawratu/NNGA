#ifndef COLLISIONFITNESS_H
#define COLLISIONFITNESS_H

#include "fitness.h"

class CollisionFitness : public Fitness
{
public:
    CollisionFitness();

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc);
};

#endif