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
        mDoubleStrings.push_back("LowerBound");
        mDoubleStrings.push_back("Value");
        mDoubleStrings.push_back("UpperBound");
        mDoubleStrings.push_back("EVWeight");
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        if(!checkParams(_pos, _dblAcc, _intAcc))
            return 0;

        if(_dblAcc["LowerBound"] > _dblAcc["Value"])
            return (_dblAcc["LowerBound"] - _dblAcc["Value"]) * _dblAcc["EVWeight"];
        else if(_dblAcc["Value"] > _dblAcc["UpperBound"])
            return (_dblAcc["Value"] - _dblAcc["UpperBound"]) * _dblAcc["EVWeight"];
        else return 0;
    }
};

#endif