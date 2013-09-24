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
    WaypointFitness(){
    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        double finalFitness = 0;

        long numWaypoints = _intAcc["NumWaypoints"];
        vector<vector3> waypoints;
        for(uint k = 0; k <= numWaypoints; k++)
            waypoints.push_back(_pos["Waypoint" + boost::lexical_cast<string>(k)]);
        
        for(map<string, vector3>::const_iterator iter = _pos.begin(); iter != _pos.end(); iter++){
            if(_intAcc[iter->first] < waypoints.size()){
                uint currentWaypoint = _intAcc[iter->first];
                
                for(; currentWaypoint < waypoints.size(); currentWaypoint++){
                    if(currentWaypoint == _intAcc[iter->first])
                        finalFitness += calcDistance(iter->second, waypoints[currentWaypoint]);
                    else finalFitness += calcDistance(waypoints[currentWaypoint - 1], waypoints[currentWaypoint]);
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
};

#endif