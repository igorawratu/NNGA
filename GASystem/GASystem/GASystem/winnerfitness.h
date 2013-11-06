#ifndef WINNERFITNESS_H
#define WINNERFITNESS_H

#include "fitness.h"
#include "common.h"

class WinnerFitness : public Fitness
{
public:
    WinnerFitness();

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc);
};

#endif