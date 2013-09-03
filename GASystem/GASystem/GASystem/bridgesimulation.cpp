#include "bridgesimulation.h"

BridgeSimulation::BridgeSimulation() : Simulation(0, 0, 0, 0, 0){

}

BridgeSimulation::~BridgeSimulation(){

}

void BridgeSimulation::iterate(){

}
    
bool BridgeSimulation::initialise(){
    return false;
}

double BridgeSimulation::fitness(vector<Fitness*> _fit){
    return 0;
}

Simulation* BridgeSimulation::getNewCopy(){
    return 0;
}