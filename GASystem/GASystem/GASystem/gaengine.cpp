#include "gaengine.h"

GAEngine::GAEngine(){
    srand(time(0));
    MutationFactory::startup();
    SelectionFactory::startup();
    CrossoverFactory::startup();
}

GAEngine::~GAEngine(){
    MutationFactory::shutdown();
    SelectionFactory::shutdown();
    CrossoverFactory::shutdown();
}

Solution GAEngine::train(GeneticAlgorithm* _geneticAlgorithm, SimulationContainer* _simulation, string _filename){
    return _geneticAlgorithm->train(_simulation, _filename);
}

void GAEngine::stopSlaves(){
    _geneticAlgorithm->stopSlaves();
}