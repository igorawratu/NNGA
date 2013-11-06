#include "expectedvaluefitness.h"

ExpectedValueFitness::ExpectedValueFitness(){
    mDoubleStrings.push_back("LowerBound");
    mDoubleStrings.push_back("Value");
    mDoubleStrings.push_back("UpperBound");
    mDoubleStrings.push_back("EVWeight");
}

double ExpectedValueFitness::evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
    if(!checkParams(_pos, _dblAcc, _intAcc))
        return 0;

    if(_dblAcc["LowerBound"] > _dblAcc["Value"])
        return (_dblAcc["LowerBound"] - _dblAcc["Value"]) * _dblAcc["EVWeight"];
    else if(_dblAcc["Value"] > _dblAcc["UpperBound"])
        return (_dblAcc["Value"] - _dblAcc["UpperBound"]) * _dblAcc["EVWeight"];
    else return 0;
}