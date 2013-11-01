#ifndef GOALPOINTFITNESS_H
#define GOALPOINTFITNESS_H

#include "fitness.h"
#include "common.h"

class GoalPointFitness : public Fitness
{
public:
    GoalPointFitness(){
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        double fitness = 0;
        double weight = _dblAcc["GPWeight"];

        double radius = _dblAcc["GoalRadius"];
        vector3 goalPoint = _pos["GoalPoint"];

        for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
            if(iter->first == "GoalPoint")
                continue;

            fitness += goalPoint.calcDistance(iter->second) - radius;
        }

        return fitness * weight;
    }
};

#endif