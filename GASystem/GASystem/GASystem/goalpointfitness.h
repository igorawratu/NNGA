#ifndef GOALPOINTFITNESS_H
#define GOALPOINTFITNESS_H

#include "fitness.h"
#include "common.h"

class GoalPointFitness : public Fitness
{
public:
    GoalPointFitness();

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc);
};

#endif