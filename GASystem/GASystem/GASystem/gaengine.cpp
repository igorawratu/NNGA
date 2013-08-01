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

Solution GAEngine::train(GeneticAlgorithm* _geneticAlgorithm, SimulationContainer* _simulation){
    return _geneticAlgorithm->train(_simulation);
}