#ifndef FINISHLINEFITNESS_H
#define FINISHLINEFITNESS_H

#include "fitness.h"
#include "common.h"

class FinishLineFitness : public Fitness
{
public:
    FinishLineFitness(){
        mDoubleStrings.push_back("FLFitnessWeight");
        mIntStrings.push_back("Positive");
        mVectorStrings.push_back("LineP1");
        mVectorStrings.push_back("LineP2");
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        if(!checkParams(_pos, _dblAcc, _intAcc))
            return 0;

        double fitness = 0;
        Line line;
        double weight = _dblAcc["FLFitnessWeight"];
        bool isPositive = _intAcc["Positive"] != 0;
        line.p1 = _pos["LineP1"];
        line.p2 = _pos["LineP2"];
        vector3 midpoint((line.p1.x + line.p2.x)/2, (line.p1.y + line.p2.y)/2, (line.p1.z + line.p2.z)/2);

        for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
            if(iter->first == "LineP1" || iter->first == "LineP2")
                continue;
            double crossVal = calcCrossVal(line.p1, line.p2, iter->second);
            bool passed = isPositive ? crossVal > 0 : crossVal < 0;
            if(!passed)
                fitness += midpoint.calcDistance(iter->second);
        }

        return fitness * weight;
    }

private:
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.z)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }
};

#endif