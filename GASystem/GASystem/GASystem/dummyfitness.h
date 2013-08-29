#ifndef DUMMYFITNESS_H
#define DUMMYFITNESS_H

#include "fitness.h"

class DummyFitness : public Fitness
{
public:
    DummyFitness(){}
    virtual ~DummyFitness(){}

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        return 0;
    }
};

#endif