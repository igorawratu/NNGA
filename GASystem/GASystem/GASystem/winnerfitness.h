#ifndef WINNERFITNESS_H
#define WINNERFITNESS_H

#include "fitness.h"
#include "common.h"

class WinnerFitness : public Fitness
{
public:
    WinnerFitness(){
        mIntStrings.push_back("Winner");
        mIntStrings.push_back("ExpectedWinner");
        mDoubleStrings.push_back("WinnerVal");
        mDoubleStrings.push_back("WinnerFitnessWeight");
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        return (_intAcc["Winner"] == _intAcc["ExpectedWinner"] ? 0 : _intAcc["WinnerVal"]) * _intAcc["WinnerFitnessWeight"];
    }
};

#endif