#ifndef FINISHLINEFITNESS_H
#define FINISHLINEFITNESS_H

#include "fitness.h"
#include "common.h"

class FinishLineFitness : public Fitness
{
public:
    FinishLineFitness();
    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc);

private:
    double calcCrossVal(vector3 a, vector3 b, vector3 c);
};

#endif