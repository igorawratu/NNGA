#include "goalpointfitness.h"

GoalPointFitness::GoalPointFitness(){
    mDoubleStrings.push_back("GPWeight");
    mDoubleStrings.push_back("GoalRadius");
    mVectorStrings.push_back("GoalPoint");
}

double GoalPointFitness::evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
    if(!checkParams(_pos, _dblAcc, _intAcc))
        return 0;

    double fitness = 0;
    double weight = _dblAcc["GPWeight"];

    double radius = _dblAcc["GoalRadius"];
    vector3 goalPoint = _pos["GoalPoint"];

    for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
        if(iter->first == "GoalPoint")
            continue;

        double dist = goalPoint.calcDistance(iter->second);

        fitness += dist <= radius ? 0 : dist - radius;
    }

    return fitness * weight;
}