#include "gaengine.h"

GAEngine::GAEngine(){
    srand(time(0));
    AlgorithmCreator::startup();
}

GAEngine::~GAEngine(){
    AlgorithmCreator::shutdown();
}

Solution GAEngine::train(GeneticAlgorithm* _geneticAlgorithm, SimulationContainer* _simulation){
    return _geneticAlgorithm->train(_simulation);
}