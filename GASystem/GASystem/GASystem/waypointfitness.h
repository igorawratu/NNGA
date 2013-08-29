#ifndef WAYPOINTFITNESS_H
#define WAYPOINTFITNESS_H

#include "fitness.h"
#include "common.h"

#include <vector>

using namespace std;

class WaypointFitness : public Fitness
{
public:
    WaypointFitness(const vector<vector3>& _waypoints){
        mWaypoints = _waypoints;
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        double finalFitness = 0;
        
        for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
            if(_intAcc[iter->first] < mWaypoints.size()){
                uint currentWaypoint = _intAcc[iter->first];
                
                for(; currentWaypoint < mWaypoints.size(); currentWaypoint++){
                    if(currentWaypoint == _intAcc[iter->first])
                        finalFitness += calcDistance(iter->second, mWaypoints[currentWaypoint]);
                    else finalFitness += calcDistance(mWaypoints[currentWaypoint - 1], mWaypoints[currentWaypoint]);
                }
            }
        }

        return finalFitness;
    }

private:
    double calcDistance(vector3 _from, vector3 _to){
        double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;
        return sqrt(x*x + y*y + z*z);
    }

private:
    vector<vector3> mWaypoints;
};

#endif