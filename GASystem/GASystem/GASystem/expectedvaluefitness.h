#ifndef EXPECTEDVALUEFITNESS_H
#define EXPECTEDVALUEFITNESS_H

#include "fitness.h"
#include "common.h"

#include <vector>
#include <boost/lexical_cast.hpp>

using namespace std;

class ExpectedValueFitness : public Fitness
{
public:
    ExpectedValueFitness(){
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        if(_dblAcc["LowerBound"] > _dblAcc["Value"])
            return (_dblAcc["LowerBound"] - _dblAcc["Value"]) * _dblAcc["EVWeight"];
        else if(_dblAcc["Value"] > _dblAcc["UpperBound"])
            return (_dblAcc["Value"] - _dblAcc["UpperBound"]) * _dblAcc["EVWeight"];
        else return 0;
    }
};

#endif