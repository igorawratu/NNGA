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
        mIntStrings.push_back("NumWaypoints");
        mIntStrings.push_back("NumAgents");
        mDoubleStrings.push_back("WPFitnessWeight");

    }

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        if(!checkParams(_pos, _dblAcc, _intAcc))
            return 0;

        double finalFitness = 0;

        long numWaypoints = _intAcc["NumWaypoints"];
        vector<vector3> waypoints;
        for(uint k = 0; k < numWaypoints; k++)
            waypoints.push_back(_pos["Waypoint" + boost::lexical_cast<string>(k)]);
        
        int numAgents = _intAcc["NumAgents"];

        for(int k = 0; k < numAgents; k++){
            string agentName = "Agent" + boost::lexical_cast<string>(k);
            double val = 0;
            if(_intAcc[agentName] < waypoints.size()){
                
                uint currentWaypoint = _intAcc[agentName];
                
                
                for(; currentWaypoint < waypoints.size(); currentWaypoint++){
                    if(currentWaypoint == _intAcc[agentName])
                        finalFitness += calcDistance(_pos[agentName], waypoints[currentWaypoint]);
                    else finalFitness += calcDistance(waypoints[currentWaypoint - 1], waypoints[currentWaypoint]);
                }
            }
        }

        return finalFitness * _dblAcc["WPFitnessWeight"];
    }

private:
    double calcDistance(vector3 _from, vector3 _to){
        double x = _to.x - _from.x, y = _to.y - _from.y, z = _to.z - _from.z;
        return sqrt(x*x + y*y + z*z);
    }
};

#endif