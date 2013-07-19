#include "algorithmcreator.h"

Selection* AlgorithmCreator::createSelectionAlgorithm(string _algorithmName, map<string, double> _parameters){
    return mSelectionFactory.create(_algorithmName, _parameters);
}

    
Mutation* AlgorithmCreator::createMutationAlgorithm(string _algorithmName, map<string, double> _parameters){
    return mMutationFactory.create(_algorithmName, _parameters);
}

Crossover* AlgorithmCreator::createCrossoverAlgorithm(string _algorithmName, map<string, double> _parameters){
    return mCrossoverFactory.create(_algorithmName, _parameters);
}


void AlgorithmCreator::startup(){
    AlgorithmCreator::initialized = true;

    AlgorithmCreator::algorithmCreator.startupSelectionAlgorithms();
    AlgorithmCreator::algorithmCreator.startupMutationAlgorithms();
    AlgorithmCreator::algorithmCreator.startupCrossoverAlgorithms();
}

void AlgorithmCreator::shutdown(){
    algorithmCreator.mMutationFactory.clear();
    algorithmCreator.mSelectionFactory.clear();
    algorithmCreator.mCrossoverFactory.clear();

    AlgorithmCreator::initialized = false;
}


void AlgorithmCreator::startupSelectionAlgorithms(){
    
}

void AlgorithmCreator::startupMutationAlgorithms(){

}

void AlgorithmCreator::startupCrossoverAlgorithms(){

}