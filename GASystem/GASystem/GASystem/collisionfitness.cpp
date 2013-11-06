#include "collisionfitness.h"

CollisionFitness::CollisionFitness(){
    mDoubleStrings.push_back("ColFitnessWeight");
    mDoubleStrings.push_back("Collisions");
}

double CollisionFitness::evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
    if(!checkParams(_pos, _dblAcc, _intAcc))
        return 0;

    return _dblAcc["Collisions"] * _dblAcc["ColFitnessWeight"];
}