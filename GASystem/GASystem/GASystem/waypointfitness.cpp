#include "waypointfitness.h"

WaypointFitness::WaypointFitness(){
    mIntStrings.push_back("NumWaypoints");
    mIntStrings.push_back("NumAgents");
    mDoubleStrings.push_back("WPFitnessWeight");

}

double WaypointFitness::evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
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
                    finalFitness += _pos[agentName].calcDistance(waypoints[currentWaypoint]);
                else finalFitness += waypoints[currentWaypoint - 1].calcDistance(waypoints[currentWaypoint]);
            }
        }
    }

    return finalFitness * _dblAcc["WPFitnessWeight"];
}