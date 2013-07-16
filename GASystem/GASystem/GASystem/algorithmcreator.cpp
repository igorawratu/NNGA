#include "algorithmcreator.h"

Selection* AlgorithmCreator::createSelectionAlgorithm(string _algorithmName, map<string, double> _parameters){
    mSelectionFactory.create(_algorithmName, _parameters);
}

    
Mutation* AlgorithmCreator::createMutationAlgorithm(string _algorithmName, map<string, double> _parameters){
    mMutationFactory.create(_algorithmName, _parameters);
}

Crossover* AlgorithmCreator::createCrossoverAlgorithm(string _algorithmName, map<string, double> _parameters){
    mCrossoverFactory.create(_algorithmName, _parameters);
}


void AlgorithmCreator::startup(){
    initialized = true;

    startupSelectionAlgorithms();
    startupMutationAlgorithms();
    startupCrossoverAlgorithms();
}

void AlgorithmCreator::shutdown(){
    mMutationFactory.clear();
    mSelectionFactory.clear();
    mCrossoverFactory.clear();
}


void AlgorithmCreator::startupSelectionAlgorithms(){
    
}

void AlgorithmCreator::startupMutationAlgorithms(){

}

void AlgorithmCreator::startupCrossoverAlgorithms(){

}