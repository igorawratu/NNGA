#ifndef FINISHLINEFITNESS_H
#define FINISHLINEFITNESS_H

#include "fitness.h"
#include "common.h"

class FinishLineFitness : public Fitness
{
public:
    FinishLineFitness(bool _isPositive, Line _line){
        mIsPositive = _isPositive;
        mLine = _line;
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        double fitness = 0;
        vector3 midpoint((mLine.p1.x + mLine.p2.x)/2, (mLine.p1.y + mLine.p2.y)/2, (mLine.p1.z + mLine.p2.z)/2);
        for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
            double crossVal = calcCrossVal(mLine.p1, mLine.p2, iter->second);
            bool passed = mIsPositive ? crossVal > 0 : crossVal < 0;
            if(!passed)
                fitness += calcDistance(iter->second, midpoint);
        }

        return fitness;
    }

private:
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.z)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }

    double calcDistance(vector3 _from, vector3 _to){
        double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;
        return sqrt(x*x + y*y + z*z);
    }

private:
    bool mIsPositive;
    Line mLine;
};

#endif