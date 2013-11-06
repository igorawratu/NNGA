#ifndef WAYPOINTFITNESS_H
#define WAYPOINTFITNESS_H

#include "fitness.h"
#include "common.h"

#include <vector>
#include <boost/lexical_cast.hpp>

using namespace std;

class WaypointFitness : public Fitness
{
public:
    WaypointFitness();

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc);
};

#endif